#include "leg_model.hpp"
#include "NodeHandler.hpp"
#include "robot_msg.hpp"
#include <iostream>
#include "boost/thread.hpp"
/* 
    This node is for estimate contact of legs, and calculate position of legs
*/
robot_msg::Motors motors_theta_data;
void motors_theta_cb(std::shared_ptr<robot_msg::Motors> msg)
{
    motors_theta_data = *msg;
}
robot_msg::Motors motors_phi_data;
void motors_phi_cb(std::shared_ptr<robot_msg::Motors> msg)
{
    motors_phi_data = *msg;
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

    boost::asio::io_context timer_io(1);
    uint sleep_us = 1000000 / freq; 

    core::NodeHandler nh(master_ip, port, local_ip);
    core::Subscriber motors_theta_sub = nh.subscriber("robot_state/motors_theta");
    core::Subscriber motors_phi_sub = nh.subscriber("robot_state/motors_phi");
    robot_msg::Legs legs_pose;
    std::vector<double> phi {0, 0, 0, 0}; // angle of whole leg
    std::vector<double> theta {17.0 / 180.0 * M_PI, 
                            17.0 / 180.0 * M_PI, 
                            17.0 / 180.0 * M_PI, 
                            17.0 / 180.0 * M_PI}; // angle between two bars, divided by 2.
    core::Publisher legs_pub = nh.publisher("robot_state/legs");
    boost::thread node_service(boost::bind(&boost::asio::io_service::run, &core::node_ios));
    while (1)
    {
        boost::asio::deadline_timer timer(timer_io, boost::posix_time::microseconds(sleep_us));
        motors_theta_sub.spinOnce(motors_theta_cb);
        motors_phi_sub.spinOnce(motors_phi_cb);
        for (int i = 0; i < 4; i++)
        {
            theta[i] = motors_theta_data.motors[i].angle;
            phi[i] = motors_phi_data.motors[i].angle;
            Cartesian pose = legs[i].position(theta[i], phi[i]);
            legs_pose.legs_pose[i].x = pose.x();
            legs_pose.legs_pose[i].y = pose.y();
            legs_pose.legs_pose[i].z = pose.z();
            // To Be Continue... covariance and contact
        }
        legs_pub.publish(legs_pose);
        timer.wait();
    }
    return 0;
}