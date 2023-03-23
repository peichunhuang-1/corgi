#include "coordinate_transform.hpp"
#include "csv_reader.hpp"

int main()
{
    mathematic::CoordinateTransformParamCartesian CTPC;
    mathematic::CoordinateTransformParamQuaternion CTPQ;
    mathematic::CoordinateTransformParamTranslation CTPT;
    mathematic::CoordinateTransformOptimizer CTOC(&CTPC);
    mathematic::CoordinateTransformOptimizer CTOQ(&CTPQ);
    mathematic::CoordinateTransformOptimizer CTOT(&CTPT);

    std::string path = "/Users/huangpeijun/Desktop/bagfiles/DataProcess/processed/attitude";
    CSV_reader imu_csv_reader;
    imu_csv_reader.load_data(path+"/imu10.csv", 0);
    CSV_reader vicon_csv_reader;
    vicon_csv_reader.load_data(path+"/vicon10.csv", 0);
    CSV_reader t265_csv_reader;
    t265_csv_reader.load_data(path+"/t26510.csv", 0);

    CTPQ.q0 = vicon_csv_reader.quaternion_data();
    CTPQ.q1 = t265_csv_reader.quaternion_data();


    std::vector<double> out(3);
    out = CTOQ.optimize(out);
    std::cout << out[0]<< out[1]<< out[2] << "\n";

    std::string path2 = "/Users/huangpeijun/Desktop/bagfiles/DataProcess/processed/position";
    CSV_reader viconp_csv_reader;
    viconp_csv_reader.load_data(path2+"/vicon4.csv", 0);
    CSV_reader t265p_csv_reader;
    t265p_csv_reader.load_data(path2+"/t2654.csv", 0);
    CTPC.x0 = viconp_csv_reader.position_data();
    CTPC.x1 = t265p_csv_reader.position_data();
    CTPT.x0 = viconp_csv_reader.position_data();
    CTPT.x1 = t265p_csv_reader.position_data();
    std::vector<double> t {0.25,0.6,0.3};
    std::vector<double> r {1.58279,-0.0205396,1.57822};
    
    for (int i = 0; i < 10; i++)
    {
        Eigen::Matrix3d R = mathematic::euler_to_rotation(r[0], r[1], r[2]);
        CTPT.rotation = R;
        t = CTOT.optimize(t);
        std::cout << t[0]<< t[1]<< t[2] << "\n";
        CTPC.translation = Eigen::Vector3d(t.data());
        r = CTOC.optimize(r);
        std::cout << r[0]<< r[1]<< r[2] << "\n";
    }
    std::ofstream file;
    std::string err_path = "/Users/huangpeijun/Desktop/bagfiles/DataProcess/processed/error/";
    file.open(err_path + "error4.csv");
    for (auto element : CTPC.err) file << sqrt(element) << "\n";
    file.close();
    return 0;
}