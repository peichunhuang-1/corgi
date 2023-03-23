#ifndef COORD_TRANS
#define COORD_TRANS

#include "nlopt.hpp"
#include "math.h"
#include "quaternion.hpp"
#include <iostream>
namespace mathematic
{

    Eigen::Matrix3d euler_to_rotation(double r, double b, double a)
    {
        double sa = sin(a);
        double ca = cos(a);
        double sb = sin(b);
        double cb = cos(b);
        double sr = sin(r);
        double cr = cos(r);
        Eigen::Matrix3d R;
        R << ca*cb, ca*sb*sr - sa*cr, ca*sb*cr+sa*sr,
            sa*cb, sa*sb*sr+ca*cr, sa*sb*cr-ca*sr,
            -sb, cb*sr, cb*cr;
        return R;
    }
    struct CoordinateTransformParamCartesian
    {
        Eigen::Vector3d translation {0, 0, 0};
        std::vector<Eigen::Vector3d> x0;
        std::vector<Eigen::Vector3d> x1;
        std::vector<double> err;
    };

    struct CoordinateTransformParamTranslation
    {
        Eigen::Matrix3d rotation;
        std::vector<Eigen::Vector3d> x0;
        std::vector<Eigen::Vector3d> x1;
        std::vector<double> err;
    };

    struct CoordinateTransformParamQuaternion
    {
        std::vector<quaternion> q0;
        std::vector<quaternion> q1;
        std::vector<double> err;
    };
    double cartesian_func(const std::vector<double> &x, std::vector<double> &grad, void *f_data);
    double quaternion_func(const std::vector<double> &x, std::vector<double> &grad, void *f_data);
    double translation_func(const std::vector<double> &x, std::vector<double> &grad, void *f_data);
    class CoordinateTransformOptimizer
    {
        public:
            nlopt::opt optimizer;
            CoordinateTransformOptimizer(CoordinateTransformParamCartesian *p);
            CoordinateTransformOptimizer(CoordinateTransformParamQuaternion *p);
            CoordinateTransformOptimizer(CoordinateTransformParamTranslation *p);
            std::vector<double> optimize(std::vector<double> u);
    };

    CoordinateTransformOptimizer::CoordinateTransformOptimizer(CoordinateTransformParamCartesian *p)
    {
        optimizer = nlopt::opt(nlopt::GN_DIRECT_L, 3);
        optimizer.set_maxeval(10000);
        std::vector<double> lb(3);
        lb[0] = -M_PI; lb[1] = -M_PI; lb[2] = -M_PI;
        std::vector<double> ub(3);
        ub[0] = M_PI; ub[1] = M_PI; ub[2] = M_PI;

        optimizer.set_lower_bounds(lb);
        optimizer.set_upper_bounds(ub);
        optimizer.set_min_objective(cartesian_func, p);
        double tol = 1e-3;
        optimizer.set_xtol_rel(tol);
        optimizer.set_force_stop(tol);
    }

    CoordinateTransformOptimizer::CoordinateTransformOptimizer(CoordinateTransformParamQuaternion *p)
    {
        optimizer = nlopt::opt(nlopt::GN_DIRECT_L, 3);
        optimizer.set_maxeval(10000);
        std::vector<double> lb(3);
        lb[0] = -M_PI; lb[1] = -M_PI; lb[2] = -M_PI;
        std::vector<double> ub(3);
        ub[0] = M_PI; ub[1] = M_PI; ub[2] = M_PI;

        optimizer.set_lower_bounds(lb);
        optimizer.set_upper_bounds(ub);
        optimizer.set_min_objective(quaternion_func, p);
        double tol = 1e-3;
        optimizer.set_xtol_rel(tol);
        optimizer.set_force_stop(tol);
    }

    CoordinateTransformOptimizer::CoordinateTransformOptimizer(CoordinateTransformParamTranslation *p)
    {
        optimizer = nlopt::opt(nlopt::GN_DIRECT_L, 3);
        optimizer.set_maxeval(10000);
        std::vector<double> lb(3);
        lb[0] = -100.0; lb[1] = -100.0; lb[2] = -100.0;
        std::vector<double> ub(3);
        ub[0] = 100.0; ub[1] = 100.0; ub[2] = 100.0;

        optimizer.set_lower_bounds(lb);
        optimizer.set_upper_bounds(ub);
        optimizer.set_min_objective(translation_func, p);
        double tol = 1e-3;
        optimizer.set_xtol_rel(tol);
        optimizer.set_force_stop(tol);
    }

    double cartesian_func(const std::vector<double> &x, std::vector<double> &grad, void *f_data)
    {
        CoordinateTransformParamCartesian* p = (CoordinateTransformParamCartesian*) (f_data);
        // a, b, r : roll, pitch, yaw
        // x0 is reference frame
        Eigen::Matrix3d R = euler_to_rotation(x[0],x[1],x[2]);
        int index = p->x1.size();
        double cost = 0;
        p->err.clear();
        for (int i = 0; i < index; i++)
        {
            Eigen::Vector3d dx = p->x0[i] - R * p->x1[i] - p->translation;
            p->err.push_back(dx.dot(dx));
            cost += p->err[i];
        }
        return cost;
    }

    double translation_func(const std::vector<double> &x, std::vector<double> &grad, void *f_data)
    {
        CoordinateTransformParamTranslation* p = (CoordinateTransformParamTranslation*) (f_data);
        // x0 is reference frame
        
        Eigen::Vector3d T;
        T << x[0],x[1],x[2];
        int index = p->x1.size();
        double cost = 0;
        p->err.clear();
        for (int i = 0; i < index; i++)
        {
            Eigen::Vector3d dx = p->x0[i] - p->rotation * p->x1[i] - T;
            p->err.push_back(dx.dot(dx));
            cost += p->err[i];
        }
        return cost;
    }

    double quaternion_func(const std::vector<double> &x, std::vector<double> &grad, void *f_data)
    {
        CoordinateTransformParamQuaternion* p = (CoordinateTransformParamQuaternion*) (f_data);
        // a, b, r : roll, pitch, yaw
        // q0 is reference frame, so R d(q1) R^t = d(q0)
        Eigen::Matrix3d R = euler_to_rotation(x[0],x[1],x[2]);
        int index = p->q1.size();
        double cost = 0;
        p->err.clear();
        Eigen::Matrix3d R0_k_ = p->q0[0].toRotationMatrix();
        Eigen::Matrix3d R1_k_ = p->q1[0].toRotationMatrix();
        for (int i = 1; i < index; i++)
        {
            Eigen::Matrix3d R0_k = p->q0[i].toRotationMatrix();
            Eigen::Matrix3d R1_k = p->q1[i].toRotationMatrix();
            Eigen::Matrix3d dR0 = R0_k_.transpose() * R0_k;
            Eigen::Matrix3d dR1 = R1_k_.transpose() * R1_k;
            p->err.push_back((dR0 - R * dR1 * R.transpose()).squaredNorm());
            cost += p->err[i];
            R0_k_ = R0_k;
            R1_k_ = R1_k;
        }
        return cost;
    }
    std::vector<double> CoordinateTransformOptimizer::optimize( std::vector<double> u)
    {
        double minf = 1e10;
        optimizer.optimize(u, minf);
        return u;
    }
}
#endif