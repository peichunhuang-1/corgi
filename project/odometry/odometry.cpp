#include "odometry.hpp"
#include "leg_model.hpp"
#include "NodeHandler.hpp"
#include "sensor_msg.hpp"
#include "robot_msg.hpp"
#include <iostream>
#include "boost/thread.hpp"

sensor_msg::Imu imu_data;
void imu_cb(std::shared_ptr<sensor_msg::Imu> msg)
{
    imu_data = *msg;
}
robot_msg::Legs legs_data;
void legs_cb(std::shared_ptr<robot_msg::Legs> msg)
{
    legs_data = *msg;
}
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

    const int legs_num = 4;
    Leg_IMU::Input imu;
    Leg_IMU::Measure<legs_num> legs_measure;
    Leg_IMU::Leg_Input_Noise<legs_num> imu_noise;
    Leg_IMU::Leg_Measure_Noise<legs_num> legs_noise;
    Leg_IMU::Leg_IMU_EKF<legs_num> odometry;
    /*
    front
    12
    43
    back
    */
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
    odometry.dt = 1.0 / freq;
    core::NodeHandler nh(master_ip, port, local_ip);
    core::Subscriber motors_theta_sub = nh.subscriber("robot_state/motors_theta");
    core::Subscriber motors_phi_sub = nh.subscriber("robot_state/motors_phi");
    core::Subscriber imu_sub = nh.subscriber("robot_state/imu");
    core::Subscriber legs_sub = nh.subscriber("robot_state/legs");
    robot_msg::State robot_odometry;
    core::Publisher odom_pub = nh.publisher("robot_state/odometry");
    std::vector<double> phi {0, 0, 0, 0}; // angle of whole leg
    std::vector<double> theta {17.0 / 180.0 * M_PI, 
                            17.0 / 180.0 * M_PI, 
                            17.0 / 180.0 * M_PI, 
                            17.0 / 180.0 * M_PI}; // angle between two bars, divided by 2.
    boost::thread node_service(boost::bind(&boost::asio::io_service::run, &core::node_ios));
    while(1)
    {
        boost::asio::deadline_timer timer(timer_io, boost::posix_time::microseconds(sleep_us));
        legs_sub.spinOnce(legs_cb);
        motors_theta_sub.spinOnce(motors_theta_cb);
        motors_phi_sub.spinOnce(motors_phi_cb);
        imu_sub.spinOnce(imu_cb);
        for (int i = 0; i < 4; i++)
        {
            theta[i] = motors_theta_data.motors[i].angle;
            phi[i] = motors_phi_data.motors[i].angle;
            legs_noise.J[i] = legs[i].Jacobian(theta[i], phi[i]);
            legs_noise.Rs[i] = Eigen::Matrix3d(legs_data.legs_pose[i].covariance);
            legs_measure.p[i] = Eigen::Vector3d(legs_data.legs_pose[i].x, legs_data.legs_pose[i].y, legs_data.legs_pose[i].z);
        }
        imu.f = Eigen::Vector3d(imu_data.accel.x, imu_data.accel.y, imu_data.accel.z);
        imu.w = Eigen::Vector3d(imu_data.angular_velocity.x, imu_data.angular_velocity.y, imu_data.angular_velocity.z);
        odometry.Qpn = imu_noise;
        odometry.Rp = legs_noise;

        odometry.update(imu);
        odometry.measure(legs_measure);

        robot_odometry.pose.x = odometry.r(0);
        robot_odometry.pose.y = odometry.r(1);
        robot_odometry.pose.z = odometry.r(2);
        robot_odometry.twist.x = odometry.v(0);
        robot_odometry.twist.y = odometry.v(1);
        robot_odometry.twist.z = odometry.v(2);
        robot_odometry.attitude.w = odometry.q(0);
        robot_odometry.attitude.x = odometry.q(1);
        robot_odometry.attitude.y = odometry.q(2);
        robot_odometry.attitude.z = odometry.q(3);
        robot_odometry.angular_velocity.x = imu_data.angular_velocity.x;
        robot_odometry.angular_velocity.y = imu_data.angular_velocity.y;
        robot_odometry.angular_velocity.z = imu_data.angular_velocity.z;

        odom_pub.publish(robot_odometry);
        timer.wait();
    }
    return 0;
}