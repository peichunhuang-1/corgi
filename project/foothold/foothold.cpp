#include "foothold.hpp"
#include "boost/thread.hpp"
#include "NodeHandler.hpp"

robot_msg::Forces force_data;
void force_cb(std::shared_ptr<robot_msg::Forces> msg)
{
    force_data = *msg;
}
robot_msg::State feedback_data;
void odometry_cb(std::shared_ptr<robot_msg::State> msg)
{
    feedback_data = *msg;
}
robot_msg::Gait gait_data;
void cpg_cb(std::shared_ptr<robot_msg::Gait> msg)
{
    gait_data = *msg;
}

int main(int argc, char* argv[])
{
    std::string master_ip = "127.0.0.1";
    std::string local_ip = "127.0.0.1";
    uint32_t port = 9999;
    double freq = 1000;
    core::ArgParse aps(argc, argv);
    aps.get("MasterIP", master_ip);
    aps.get("MasterPort", port);
    aps.get("LocalIP", local_ip);
    aps.get("Frequency", freq);

    Eigen::Matrix3d L_to_B; 
    L_to_B << 0., 1., 0., 0., 0., 1., 1., 0., 0.;
    std::vector<LinkLeg::Leg> legs;
    legs.resize(4);
    Cartesian v1(0.222, 0.1995, -0.1);
    legs[0] = LinkLeg::Leg(0.1, L_to_B, v1);
    Cartesian v2(0.222, -0.1995, -0.1);
    legs[1] = LinkLeg::Leg(0.1, L_to_B, v2);
    Cartesian v3(-0.222, -0.1995, -0.1);
    legs[2] = LinkLeg::Leg(0.1, L_to_B, v3);
    Cartesian v4(-0.222, 0.1995, -0.1);
    legs[3] = LinkLeg::Leg(0.1, L_to_B, v4);

    double mass = 20.0;
    double height = 0.15;
    double width = 0.3;
    double length = 0.5;
    Eigen::Matrix3d inertia; 
    inertia << mass*(height * height + width * width), 0, 0,
                0, mass*(height * height + length * length), 0,
                0, 0, mass*(width * width + length * length);
    State s;
    s.position = Eigen::Vector3d(0, 0, 0);
    s.velocity = Eigen::Vector3d(0, 0, 0);
    s.angular_velocity = Eigen::Vector3d(0, 0, 0);
    s.attitude = Eigen::Vector4d(1, 0, 0, 0);

    RigidBody robot;
    robot.mass = mass;
    robot.inertia = inertia;
    robot.state = s;
    robot.joints["centroid"] = Joint(0, 0, 0);
    robot.joints["0"] = Joint(0.222, 0.1995, -0.1);
    robot.joints["1"] = Joint(0.222, -0.1995, -0.1);
    robot.joints["2"] = Joint(-0.222, -0.1995, -0.1);
    robot.joints["3"] = Joint(-0.222, 0.1995, -0.1);

    Foothold foothold_planner;
    foothold_planner.duration = 1. / freq;
    foothold_planner.legs = legs;
    foothold_planner.ground_height = 0;

    std::vector<std::string> jts_name = {"centroid", "0", "1", "2", "3"};
    std::vector<Eigen::Vector3d> forces; 
    forces = {Eigen::Vector3d(0, 0, -9.81), 
                Eigen::Vector3d(0, 0, 0),
                Eigen::Vector3d(0, 0, 0),
                Eigen::Vector3d(0, 0, 0),
                Eigen::Vector3d(0, 0, 0)};

    boost::asio::io_context timer_io(1);
    uint sleep_us = 1000000 / freq; 

    core::NodeHandler nh(master_ip, port, local_ip);
    core::Subscriber force_sub = nh.subscriber("robot_state/force_control");
    core::Subscriber odometry_sub = nh.subscriber("robot_state/odometry");
    core::Subscriber cpg_sub = nh.subscriber("robot_state/cpg/phase");
    robot_msg::Motors motors_theta_data;
    robot_msg::Motors motors_beta_data;
    gait_data.contact[0] = true;
    gait_data.contact[1] = true;

    core::Publisher theta_pub = nh.publisher("robot_state/control/motors_theta");
    core::Publisher beta_pub = nh.publisher("robot_state/control/motors_beta");
    
    boost::thread node_service(boost::bind(&boost::asio::io_service::run, &core::node_ios));
    while (1)
    {
        boost::asio::deadline_timer timer(timer_io, boost::posix_time::microseconds(sleep_us));
        force_sub.spinOnce(force_cb);
        odometry_sub.spinOnce(odometry_cb);
        cpg_sub.spinOnce(cpg_cb);

        s.position = Eigen::Vector3d(feedback_data.pose.x, feedback_data.pose.y, feedback_data.pose.z);
        s.velocity = Eigen::Vector3d(feedback_data.twist.x, feedback_data.twist.y, feedback_data.twist.z);
        s.attitude = Eigen::Vector4d(feedback_data.attitude.w, feedback_data.attitude.x, feedback_data.attitude.y, feedback_data.attitude.z);
        s.angular_velocity = Eigen::Vector3d(feedback_data.angular_velocity.x, feedback_data.angular_velocity.y, feedback_data.angular_velocity.z);
        robot.state = s;

        forces[1] = Eigen::Vector3d(force_data.forces[0].vector[0],force_data.forces[0].vector[1],force_data.forces[0].vector[2]);
        forces[2] = Eigen::Vector3d(force_data.forces[1].vector[0],force_data.forces[1].vector[1],force_data.forces[1].vector[2]);
        forces[3] = Eigen::Vector3d(force_data.forces[2].vector[0],force_data.forces[2].vector[1],force_data.forces[2].vector[2]);
        forces[4] = Eigen::Vector3d(force_data.forces[3].vector[0],force_data.forces[3].vector[1],force_data.forces[3].vector[2]);

        foothold_planner.plan(robot, jts_name, forces, gait_data);

        motors_theta_data.motors[0].angle = foothold_planner.motor_angles[0].first;
        motors_theta_data.motors[1].angle = foothold_planner.motor_angles[1].first;
        motors_theta_data.motors[2].angle = foothold_planner.motor_angles[2].first;
        motors_theta_data.motors[3].angle = foothold_planner.motor_angles[3].first;

        motors_beta_data.motors[0].angle = foothold_planner.motor_angles[0].second;
        motors_beta_data.motors[1].angle = foothold_planner.motor_angles[1].second;
        motors_beta_data.motors[2].angle = foothold_planner.motor_angles[2].second;
        motors_beta_data.motors[3].angle = foothold_planner.motor_angles[3].second;

        theta_pub.publish(motors_theta_data);
        beta_pub.publish(motors_beta_data);

        timer.wait();
    }
    
    return 0;
}