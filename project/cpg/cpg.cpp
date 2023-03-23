#include "NodeHandler.hpp"
#include "cpg.hpp"
#include "robot_msg.hpp"
#include "boost/thread.hpp"

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

    Eigen::Matrix<bool, 9, 4> ban_list; ban_list << 0, 0, 1, 1,
                                        0, 1, 1, 0,
                                        1 ,1 ,0 ,0,
                                        1 ,0 ,0 ,1,
                                        1 ,1 ,1 ,0,
                                        0 ,1 ,1 ,1,
                                        1 ,0 ,1 ,1,
                                        1 ,1 ,0 ,1,
                                        1 ,1 ,1 ,1;
    Eigen::Matrix<double, 4, 4> potential; potential << 0, 1, 1, 1,
                                        1, 0, 1, 1,
                                        1 ,1 ,0 ,1,
                                        1 ,1 ,1 ,0;
    Eigen::Vector4d phase; phase << 1e-1, 1e-1, 1e-1, 1e-1;
    double c_gain = 2;
    double p_gain = 60;
    Kuramoto_cpg cpg(ban_list, potential);
    double zeta = 0.5;
    double dt = 1.0 / freq;
    cpg(c_gain, p_gain, zeta);
    cpg.phase(phase);

    boost::asio::io_context timer_io(1);
    uint sleep_us = 1000000 / freq; 
    
    core::NodeHandler nh(master_ip, port, local_ip);
    core::Subscriber sub = nh.subscriber("robot_state/cpg/ctrl"); 
    robot_msg::Gait phase_data;
    core::Publisher pub = nh.publisher("robot_state/cpg/phase"); 
    boost::thread node_service(boost::bind(&boost::asio::io_service::run, &core::node_ios));

    usleep(100000);
    while (1)
    {
        boost::asio::deadline_timer timer(timer_io, boost::posix_time::microseconds(sleep_us));
        sub.spinOnce(cpg_cb);
        cpg(c_gain, p_gain, gait_data.zeta);
        cpg.phase_generator(gait_data.type, dt);
        phase_data.type = gait_data.type;
        phase_data.gait_height = gait_data.gait_height;
        phase_data.gait_length = gait_data.gait_length;
        phase_data.zeta = gait_data.zeta;
        for (int i = 0; i < 4; i ++)
        {
            phase_data.phase[i] = cpg.phase()(i);
            phase_data.contact[i] = cpg.contact()(i);
        }
        pub.publish(phase_data);
        timer.wait();
    }
    return 0;
}