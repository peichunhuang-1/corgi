#include "NodeHandler.hpp"
#include "imu.hpp"
#include "sensor_msg.hpp"

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


    boost::asio::io_context timer_io(1);
    uint sleep_us = 1000000 / freq; 
    CX5_AHRS imu("/dev/ttyACM0", 921600, 500, 500);
    sensor_msg::Imu imu_raw_data;
    boost::thread* imu_thread;
    imu_thread = new boost::thread(boost::bind(&CX5_AHRS::setup, &imu));
    core::NodeHandler nh(master_ip, port, local_ip);;
    core::Publisher pub = nh.publisher("robot_state/imu"); // imu msg
    boost::thread node_service(boost::bind(&boost::asio::io_service::run, &core::node_ios));

    usleep(100000);
    while (1)
    {
        boost::asio::deadline_timer timer(timer_io, boost::posix_time::microseconds(sleep_us));
        imu_raw_data.accel.x = imu.acceleration.x;
        imu_raw_data.accel.y = imu.acceleration.y;
        imu_raw_data.accel.z = imu.acceleration.z;

        imu_raw_data.angular_velocity.x = imu.angular_velocity.x;
        imu_raw_data.angular_velocity.y = imu.angular_velocity.y;
        imu_raw_data.angular_velocity.z = imu.angular_velocity.z;
        imu_raw_data.attitude.w = imu.quaternion.w;
        imu_raw_data.attitude.x = imu.quaternion.x;
        imu_raw_data.attitude.y = imu.quaternion.y;
        imu_raw_data.attitude.z = imu.quaternion.z;
        // std::cout << "acceleration:\t" << imu_raw_data.acceleration()[0] << "\t" << imu_raw_data.acceleration()[1] << "\t" << imu_raw_data.acceleration()[2] << "\n";
        pub.publish(imu_raw_data);
        timer.wait();
    }
    return 0;
}