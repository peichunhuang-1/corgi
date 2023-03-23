#ifndef ODOMETRY
#define ODOMETRY
#include "Eigen/Dense"
#include "quaternion.hpp"
namespace Leg_IMU
{
    struct Input
    {
        public:
            Eigen::Vector<double, 3> f;
            Eigen::Vector<double, 3> w;
    };
    template<int N>
    struct Measure
    {
        public:
            std::array<Eigen::Vector<double, 3>, N> p;
    };
    template<int N>
    struct Leg_Input_Noise
    {
        public:
            std::array<Eigen::Matrix3d, N> Q;
            Eigen::Matrix<double, 3, 3> Qf;
            Eigen::Matrix<double, 3, 3> Qbf;
            Eigen::Matrix<double, 3, 3> Qw;
            Eigen::Matrix<double, 3, 3> Qbw;
    };

    template<int N>
    struct Leg_Measure_Noise
    {
        public:
            std::array<Eigen::Matrix3d, N> Rs;
            std::array<Eigen::Matrix2d, N> Ra;
            std::array<Eigen::Matrix<double, 3, 2>, N > J;
    };

    template<int N>
    class Leg_IMU_EKF
    {
        private:
            Eigen::Matrix<double, 3*(N+5), 3*(N+5)> F;
            Eigen::Matrix<double, 3*(N+5), 3*(N+5)> P;
            Eigen::Matrix<double, 3*(N+5), 3*(N+5)> Q;

            
            Eigen::Matrix<double, 3*N, 3*N> Qp; // combine Qp to a matrix
            
            Eigen::Matrix<double, 3*(N), 3*(N+5)> H;
            Eigen::Matrix<double, 3*N, 3*N> R;
            Eigen::Matrix<double, 3*N, 3*N> S;
            Eigen::Matrix<double, 3*(N+5), 3*N> K;

            Eigen::Vector<double, 12> y;

            

        public:
            Leg_IMU_EKF() {}
            double dt;
            // state estimated
            Eigen::Vector<double, 3> r;
            Eigen::Vector<double, 3> v;
            Eigen::Vector<double, 4> q;
            Measure<N> p;
            Eigen::Vector<double, 3> bf;
            Eigen::Vector<double, 3> bw;
            // Noise Covaraince
            Leg_Input_Noise<N> Qpn; // 3 x 3 (4 or 6 or ...)
            Leg_Measure_Noise<N> Rp;

            // function
            void update(Input u);
            void measure(Measure<N> m);
    };
    template<int N>
    void Leg_IMU_EKF<N>::update(Input u)
    {
        Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
        Eigen::MatrixXd In = Eigen::MatrixXd::Identity(3*N, 3*N);
        Eigen::Matrix3d O3 = Eigen::Matrix3d::Zero();
        Eigen::MatrixXd O3xn = Eigen::MatrixXd::Zero(3, 3*N);
        Eigen::MatrixXd Onx3 = Eigen::MatrixXd::Zero(3*N, 3);
        Eigen::Matrix3d G0 = mathematic::Gamma<3>(u.w-bw, dt, 0);
        Eigen::Matrix3d G1 = mathematic::Gamma<3>(u.w-bw, dt, 1);
        Eigen::Matrix3d G2 = mathematic::Gamma<3>(u.w-bw, dt, 2);
        Eigen::Matrix3d G3 = mathematic::Gamma<3>(u.w-bw, dt, 3);

        Eigen::Matrix3d R_plus = mathematic::quaternion(q).toRotationMatrix();
        
        F << I3 , I3 * dt , dt * dt * 0.5 * R_plus * Eigen::skew3(u.f - bf), O3xn, dt * dt * 0.5 * R_plus, O3,
             O3, I3, - dt * R_plus * Eigen::skew3(u.f - bf), O3xn, dt * R_plus, O3,
             O3, O3, G0.transpose(), O3xn, O3, G1.transpose(),
             Onx3, Onx3, Onx3, In, Onx3, Onx3, 
             O3, O3, O3, O3xn, I3, O3,
             O3, O3, O3, O3xn, O3, I3;
        Qp = Eigen::MatrixXd::Zero(3*N, 3*N);
        for (int i = 0; i < N; i++)
            (Qp.template block<3, 3> (3*i, 3*i)) = R_plus.transpose() * Qpn.Q[i] * R_plus;
        Q << dt * dt * dt / 3.0 * Qpn.Qf + pow(dt, 5) / 20.0 * Qpn.Qbf, dt * dt * 0.5 * Qpn.Qf + pow(dt, 4) * 0.125 * Qpn.Qbf, O3, O3xn, - dt * dt * dt / 6.0 * R_plus * Qpn.Qbf, O3,
             dt * dt * 0.5 * Qpn.Qf + pow(dt, 4) * 0.125 * Qpn.Qbf, dt * Qpn.Qf + dt * dt * dt / 3.0 * Qpn.Qbf, O3, O3xn, - dt * dt * 0.5 * R_plus * Qpn.Qbf, O3,
             O3, O3, dt * Qpn.Qw + (G3 + G3.transpose()) * Qpn.Qbw, O3xn, O3, -G2.transpose() * Qpn.Qbw,
             Onx3, Onx3, Onx3, dt * Qp, Onx3, Onx3,
             O3, O3, O3, O3xn, dt * Qpn.Qbf, O3,
             O3, O3, O3, O3xn, O3, dt * Qpn.Qbw;
        P = F * P * F.transpose() + Q;

        r = r + dt * v + dt * dt * 0.5 * (R_plus.transpose() * (u.f - bf) + Eigen::Vector3d(0, 0, -9.80665));
        v = v + dt * (R_plus.transpose() * (u.f - bf));
        q = (mathematic::zeta(u.w * dt) * q).q(); 
    }
    template<int N>
    void Leg_IMU_EKF<N>::measure(Measure<N> m)
    {
        Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d O3 = Eigen::Matrix3d::Zero();
        Eigen::MatrixXd O3xn = Eigen::MatrixXd::Zero(3, 3*(N));
        Eigen::Matrix3d R_minus = mathematic::quaternion(q).toRotationMatrix();
        
        for (int i = 0; i < N; i++)
            (y.template segment<3> (i*3)) = m.p[i] - R_minus.transpose() * (p.p[i] - r);
        Eigen::Matrix<double, 3, 3*(N+5)> H_row_i; 
        for (int i = 0; i < N; i++)
        {
            H_row_i << - R_minus, O3, Eigen::skew3(R_minus.transpose() * (p.p[i] - r)), O3xn, O3, O3;
            (H_row_i.template block<3, 3>(0, 3*(i+3))) = R_minus;
            H.template block<3, 3*(N+5)>(3*i, 0) = H_row_i;
        }
        for (int i = 0; i < N; i++)
            (R.template block<3, 3> (3*i, 3*i)) = Rp.Rs[i] + Rp.J[i] * Rp.Ra[i] * Rp.J[i].transpose();
        S = H * P * H.transpose() + R;
        K = P * H.transpose() * S.inverse();
        Eigen::Vector<double, 3*(N+5)> dx = K * y;
        P = (Eigen::MatrixXd::Identity(3*(N+5), 3*(N+5)) - K * H) * P;
        r = r + dx.template segment<3> (0);
        v = v + dx.template segment<3> (3);
        q = (mathematic::zeta(dx.template segment<3> (6)) * q).q();
        for (int i = 0; i < N; i++)
            p.p[i] = p.p[i] + dx.template segment<3> (9+3*i);
        bf = bf + dx.template segment<3> ((3+N)*3);
        bw = bw + dx.template segment<3> ((4+N)*3);
    }
}
#endif