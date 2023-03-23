#ifndef FOOTHOLD
#define FOOTHOLD
#include "Eigen/Dense"
#include "rigid_body.hpp"
#include "robot_msg.hpp"
#include "leg_model.hpp"
#include "quaternion.hpp"
#include "map"
#include "coordinate.hpp"
#include <vector>
class Foothold
{
    public:
        double duration;
        std::pair<double, double> motor_angles[4];
        std::vector<LinkLeg::Leg> legs;
        std::vector<Eigen::Vector3d> legs_position;
        std::vector<Eigen::Vector3d> footprint;
        Eigen::Matrix3d R;
        Eigen::Vector3d last_position;
        double ground_height = 0;
        Foothold() {legs.resize(4); legs_position.resize(4); footprint.resize(4);}
        void plan(RigidBody robot, std::vector<std::string> jts_name, std::vector<Eigen::Vector3d> forces, robot_msg::Gait gait_control);
};  

void Foothold::plan(RigidBody robot, std::vector<std::string> jts_name, std::vector<Eigen::Vector3d> forces, robot_msg::Gait gait_control)
{
    State last_state = robot.state;
    Eigen::Matrix3d C_l = mathematic::quaternion(robot.state.attitude).toRotationMatrix();
    robot.apply_forces(jts_name, forces, duration);
    Eigen::Matrix3d C_r = mathematic::quaternion(robot.state.attitude).toRotationMatrix();
    for (int i = 0; i < 4; i++)
    {
        if (gait_control.contact[i]) // if not contact
        {
            
            Eigen::Matrix3d A;
            Eigen::Vector3d v1 = R.col(1);
            Eigen::Vector3d v2= Eigen::Vector3d(0, 0, 1);
            Eigen::Vector3d v3 = R.col(0);
            double swing_duration, T;

            switch (gait_control.type)
            {
            case robot_msg::WALK:
                swing_duration = M_PI/2. - 0.2;
                T = 1.0 / gait_control.zeta * 5.0 / 8.0;
                break;
            case robot_msg::TROT:
                swing_duration = M_PI - 0.4;
                T = 1.0 / gait_control.zeta * 3.0 / 4.0;
                break;
            case robot_msg::STOP:
                break;
            default:
                swing_duration = M_PI/2. - 0.2;
                break;
            }
            A.row(0) = v1.transpose();
            A.row(1) = v2.transpose();
            A.row(2) = v3.transpose();
            Eigen::Vector3d leg_joint = last_position + R * Eigen::Vector3d(legs[i].vec.x(), legs[i].vec.y(), legs[i].vec.z());
            Eigen::Vector3d b = Eigen::Vector3d(v1.dot(leg_joint), ground_height, v3.dot(leg_joint + T * robot.state.velocity));
            legs_position[i] = A.inverse() * b; // world frame
            
            double theta = 1 / swing_duration * (fmod(gait_control.phase[i], 2.0 * M_PI) - M_PI + 0.5 * swing_duration);
            Eigen::Vector3d pose = footprint[i] + (legs_position[i] - footprint[i]) * theta;
            pose(2) += 4 * theta * (1 - theta) * gait_control.gait_height;
            pose = C_l.transpose() * (pose - last_state.position);
            motor_angles[i] = legs[i].inv(Cartesian(pose(0), pose(1), pose(2)));
        }
        else
        {
            legs_position[i] = C_r.transpose() * (last_state.position - robot.state.position + C_l * legs_position[i]); // body frame
            footprint[i] = last_state.position + C_l * legs_position[i]; // world frame
            last_position = last_state.position;
            R = C_l;
            motor_angles[i] = legs[i].inv(Cartesian(legs_position[i](0), legs_position[i](1), legs_position[i](2)));
        }
    }
}

#endif