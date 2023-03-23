#include "force_optimizer.hpp"
#include "nlopt.hpp"
#include "rigid_body.hpp"
#include <iostream>
#include "boost/thread.hpp"
#include "robot_msg.hpp"
#include "NodeHandler.hpp"


robot_msg::State feedback_data;
void odometry_cb(std::shared_ptr<robot_msg::State> msg)
{
    feedback_data = *msg;
}

robot_msg::State control_data;
void control_cb(std::shared_ptr<robot_msg::State> msg)
{
    control_data = *msg;
}

robot_msg::Legs legs_data;
void legs_cb(std::shared_ptr<robot_msg::Legs> msg)
{
    legs_data = *msg;
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

    
    /*
    front
    12
    43
    back
    */
    /* robot data */
    Eigen::Vector<double, 12> p {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
    Eigen::Vector<double, 12> q {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
    q = 1e-5 * q;
    p = 1e5 * p;
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

    // optimizer data
    std::vector<double> mask {1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1};
    std::vector<Feature> landforms(4);
    ForceOptimizer fopt_p(p, q, mask);
    fopt_p.land_form = landforms;
    fopt_p.duration = 1.0 / freq;
    fopt_p.rigid_body.mass = mass;
    fopt_p.rigid_body.inertia = inertia;
    fopt_p.rigid_body.state = s;
    fopt_p.ctrl = s;
    fopt_p.rigid_body.joints["centroid"] = Joint(0, 0, 0);
    fopt_p.rigid_body.joints["0"] = Joint(0.222, 0.1995, -0.1);
    fopt_p.rigid_body.joints["1"] = Joint(0.222, -0.1995, -0.1);
    fopt_p.rigid_body.joints["2"] = Joint(-0.222, -0.1995, -0.1);
    fopt_p.rigid_body.joints["3"] = Joint(-0.222, 0.1995, -0.1);
    nlopt::opt fopt = Optimizer(&fopt_p);
    std::vector<double> u {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    double minf = 0;

    boost::asio::io_context timer_io(1);
    uint sleep_us = 1000000 / freq; 

    core::NodeHandler nh(master_ip, port, local_ip);
    core::Subscriber control_sub = nh.subscriber("robot_state/control_position");
    core::Subscriber odom_sub = nh.subscriber("robot_state/odometry");
    core::Subscriber legs_sub = nh.subscriber("robot_state/legs");
    robot_msg::Forces force_data;
    core::Publisher force_pub = nh.publisher("robot_state/force_control");

    boost::thread node_service(boost::bind(&boost::asio::io_service::run, &core::node_ios));
    while(1)
    {
        boost::asio::deadline_timer timer(timer_io, boost::posix_time::microseconds(sleep_us));
        control_sub.spinOnce(control_cb);
        odom_sub.spinOnce(odometry_cb);
        legs_sub.spinOnce(legs_cb);
        s.position = Eigen::Vector3d(feedback_data.pose.x, feedback_data.pose.y, feedback_data.pose.z);
        s.velocity = Eigen::Vector3d(feedback_data.twist.x, feedback_data.twist.y, feedback_data.twist.z);
        s.angular_velocity = Eigen::Vector3d(feedback_data.angular_velocity.x, feedback_data.angular_velocity.y, feedback_data.angular_velocity.z);
        s.attitude = Eigen::Vector4d(feedback_data.attitude.w, feedback_data.attitude.x, feedback_data.attitude.y, feedback_data.attitude.z);
        fopt_p.rigid_body.state = s;
        s.position = Eigen::Vector3d(control_data.pose.x, control_data.pose.y, control_data.pose.z);
        s.velocity = Eigen::Vector3d(control_data.twist.x, control_data.twist.y, control_data.twist.z);
        s.angular_velocity = Eigen::Vector3d(control_data.angular_velocity.x, control_data.angular_velocity.y, control_data.angular_velocity.z);
        s.attitude = Eigen::Vector4d(control_data.attitude.w, control_data.attitude.x, control_data.attitude.y, control_data.attitude.z);
        fopt_p.ctrl = s;
        Eigen::Matrix<double, 12, 12> J;
        J << Jacobian(fopt_p.rigid_body, "0", fopt_p.duration), 
            Jacobian(fopt_p.rigid_body, "1", fopt_p.duration),
            Jacobian(fopt_p.rigid_body, "2", fopt_p.duration),
            Jacobian(fopt_p.rigid_body, "3", fopt_p.duration); 
        fopt_p.J = J;
        fopt_p.contact[0] = legs_data.legs_contact[0];
        fopt_p.contact[1] = legs_data.legs_contact[1];
        fopt_p.contact[2] = legs_data.legs_contact[2];
        fopt_p.contact[3] = legs_data.legs_contact[3];
        fopt.optimize(u, minf);
        force_data.forces[0].vector[0] = u[0];
        force_data.forces[0].vector[1] = u[1];
        force_data.forces[0].vector[2] = u[2];
        force_data.forces[1].vector[0] = u[3];
        force_data.forces[1].vector[1] = u[4];
        force_data.forces[1].vector[2] = u[5];
        force_data.forces[2].vector[0] = u[6];
        force_data.forces[2].vector[1] = u[7];
        force_data.forces[2].vector[2] = u[8];
        force_data.forces[3].vector[0] = u[9];
        force_data.forces[3].vector[1] = u[10];
        force_data.forces[3].vector[2] = u[11];
        force_pub.publish(force_data);

        timer.wait();
    }
    return 0;
}