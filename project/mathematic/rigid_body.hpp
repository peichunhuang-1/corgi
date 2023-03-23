#ifndef RIGID_BODY_KINEMATIC_HPP
#define RIGID_BODY_KINEMATIC_HPP
#include <quaternion.hpp>
#include <Eigen/Dense>
#include <vector>
#include <map>
struct Joint
{
    public:
        Eigen::Vector3d position;
        Joint(){}
        Joint(double x, double y, double z)
        {
            position(0) = x;
            position(1) = y;
            position(2) = z;
        }  
};

struct State
{
    public:
        Eigen::Vector3d position = {0, 0, 0};
        Eigen::Vector3d velocity = {0, 0, 0};
        Eigen::Vector3d angular_velocity = {0, 0, 0};
        Eigen::Vector4d attitude = {1, 0, 0, 0}; // w x y z
};  

struct RigidBody
{
    public:
        Eigen::Matrix3d inertia;
        double mass;
        State state;
        std::map<std::string, Joint> joints;
        void apply_force(std::string joint, Eigen::Vector3d force, double duration);
        void apply_torque(Eigen::Vector3d torque, double duration);
        void apply_forces(std::vector<std::string> jts, std::vector<Eigen::Vector3d> forces, double duration);
};
void RigidBody::apply_force(std::string joint, Eigen::Vector3d force, double duration)
{
    Eigen::Vector3d torque = joints[joint].position.cross(force);
    force = mathematic::quaternion(state.attitude).toRotationMatrix() * force;
    this->apply_torque(torque, duration);
    Eigen::Vector3d acceleration = force / mass;
    state.position += state.velocity * duration + 0.5 * duration * acceleration * duration;
    state.velocity += acceleration * duration;
}

void RigidBody::apply_forces(std::vector<std::string> jts, std::vector<Eigen::Vector3d> forces, double duration)
{
    Eigen::Vector3d torque;
    Eigen::Matrix3d C = mathematic::quaternion(state.attitude).toRotationMatrix();
    for (int i = 0; i < jts.size(); i++)
    {
        torque += joints[jts[i]].position.cross(forces[i]);
        forces[i] = C * forces[i];
    }
    this->apply_torque(torque, duration);
    for (auto force : forces)
    {
        Eigen::Vector3d acceleration = force / mass;
        state.position += state.velocity * duration + 0.5 * duration * acceleration * duration;
        state.velocity += acceleration * duration;
    }
}

void RigidBody::apply_torque(Eigen::Vector3d torque, double duration)
{
    Eigen::Vector3d alpha = inertia.inverse() * torque - inertia.inverse() * Eigen::skew3(state.angular_velocity) * inertia * state.angular_velocity;
    
    state.attitude = (mathematic::zeta(state.angular_velocity * duration + 0.5 * alpha * duration * duration) * mathematic::quaternion(state.attitude)).q();
    state.angular_velocity += alpha * duration;
}
Eigen::Matrix<double, 12, 3> Jacobian(RigidBody r, std::string joint, double duration)
{
    // jacobian calculate {r, v, phi, omega}
    Eigen::Matrix3d C = mathematic::quaternion(r.state.attitude).toRotationMatrix();
    Eigen::Matrix<double, 12, 3> j;
    j << duration * duration * 0.5 / r.mass * Eigen::Matrix3d::Identity() * C, duration / r.mass * Eigen::Matrix3d::Identity() * C
      , duration * duration * 0.5 * r.inertia.inverse() * Eigen::skew3(r.joints[joint].position) ,  duration * r.inertia.inverse() * Eigen::skew3(r.joints[joint].position) ;
    return j;
}

Eigen::Vector<double, 12> Diff(State l, State r) // l - r
{
    Eigen::Vector3d d_p, d_v, d_phi, d_omega;
    Eigen::Vector<double, 12> delta;
    d_p = l.position - r.position;
    d_v = l.velocity - r.velocity;
    d_omega = l.angular_velocity - r.angular_velocity;
    mathematic::quaternion q_r = mathematic::quaternion(r.attitude);
    q_r.inverse();
    mathematic::quaternion d_q = mathematic::quaternion(l.attitude) * q_r;
    d_phi = d_q.toEuler();
    delta << d_p, d_v, d_phi, d_omega;
    return delta;
}

struct Feature
{
    public:
        double Fs = 0.01; // static friction coefficient
        Eigen::Vector3d vector = {0, 0, 1.0};
};

#endif