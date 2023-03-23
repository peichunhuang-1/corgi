#ifndef OPT_HPP
#define OPT_HPP
#include "Eigen/Dense"
#include "rigid_body.hpp"
#include "nlopt.hpp"


class ForceOptimizer
{
    public:
        ForceOptimizer() {}
        ForceOptimizer(Eigen::Vector<double, 12> p, Eigen::Vector<double, 12> q, std::vector<double> m);
        Eigen::Matrix<double, 12, 12> P;
        Eigen::Matrix<double, 12, 12> Q;
        std::vector<double> input_mask;
        Eigen::Matrix<double, 12, 12> J;
        Eigen::Vector3d gravity = {0, 0, -9.81};
        State ctrl;
        RigidBody rigid_body;
        std::vector<Feature> land_form;
        double duration;
        double constraints[8];
        bool contact[4] = {false, false, false, false}; // false = contact
};
ForceOptimizer::ForceOptimizer(Eigen::Vector<double, 12> p, Eigen::Vector<double, 12> q ,std::vector<double> mask)
{
    P = p.matrix().asDiagonal();
    Q = q.matrix().asDiagonal();
    input_mask = mask ;
}

double opt_func(const std::vector<double> &x, std::vector<double> &grad, void *f_data)
{
    ForceOptimizer *fop = (ForceOptimizer*) (f_data);
    RigidBody r = fop->rigid_body;

    Eigen::Matrix3d C = mathematic::quaternion(r.state.attitude).toRotationMatrix();
    std::vector<double> input(12);
    std::transform(x.begin(), x.end(), fop->input_mask.begin(), input.begin(), std::multiplies<double>() );
    std::vector<Eigen::Vector3d> forces(4);
    Eigen::Vector<double, 12> u;
    for (int i = 0; i < 4; i++)
    {
        if (fop->contact[i]) forces[i] = Eigen::Vector3d(0, 0, 0);
        else forces[i] = Eigen::Vector3d(input.data()+i*3);
    }
    u << forces[0], forces[1], forces[2], forces[3];
    r.apply_force("centroid", fop->rigid_body.mass * fop->gravity, fop->duration);
    std::vector<std::string> joints_name = {"0", "1", "2", "3"};
    r.apply_forces(joints_name, forces, fop->duration);
    
    // for (int i = 0; i < 4; i ++)
    //     r.apply_force(std::to_string(i), Eigen::Vector3d(input.data() + i * 3), fop->duration);
    Eigen::Vector<double, 12> s_err = Diff(fop->ctrl, r.state);
    Eigen::Vector<double, 12> gradient = -2 * (s_err.transpose() * fop->P * fop->J - u.transpose() * fop->Q);
    for (int i = 0; i < 12; i++) 
        grad[i] = gradient(i);
    for (int i = 0; i < 4; i++)
    {
        if (fop->contact[i]) 
        {
            grad[i*3] = 0;
            grad[i*3+1] = 0;
            grad[i*3+2] = 0;
        }
    }
    for (int i = 0; i < 4; i++)
    {
        Eigen::Vector3d force = Eigen::Vector3d(input.data() + i * 3);
        force = C * force;
        double fs = fop->land_form[i].Fs;

        fop->constraints[i] = - force.dot(fop->land_form[i].vector);
        fop->constraints[i*2] = force.dot(force) - (1+fs*fs) * fop->constraints[i] * fop->constraints[i] - 1e-2;
        fop->constraints[i] -= 1e-2;
    }
    double result = s_err.transpose() * fop->P * s_err;
    result += u.transpose() * fop->Q * u;
    return result;
}

void constraints(unsigned m, double *result, unsigned n, const double* x, double* grad, void* f_data)
{
    ForceOptimizer *fop = (ForceOptimizer*) (f_data);
    for ( int i = 0; i < m; i ++)
    {
        result[i] = fop->constraints[i];
    }
    return;
}


nlopt::opt Optimizer(ForceOptimizer* fop)
{
    nlopt::opt fopt = nlopt::opt(nlopt::LD_MMA, 12);
    fopt.set_param("inner_maxeval", 5);
    fopt.set_maxeval(30);
    std::vector<double> lb {-200, 0, -200, 
                            -200, 0, -200,
                            -200, 0, -200, 
                            -200, 0, -200};
    
    std::vector<double> ub {200, 0, 200, 
                            200, 0, 200,
                            200, 0, 200, 
                            200, 0, 200};

    fopt.set_lower_bounds(lb);
    fopt.set_upper_bounds(ub);
    fopt.set_min_objective(opt_func, fop);
    double tol = 1e-3;
    std::vector<double> tols {1e-3, 1e-3, 1e-3, 1e-3};

    fopt.add_inequality_mconstraint(constraints, fop, tols);
    fopt.set_xtol_rel(tol);
    fopt.set_force_stop(tol);
    return fopt;
}

#endif