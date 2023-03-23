#ifndef QUATERNION
#define QUATERNION
#include <cmath>
#include "Eigen/Dense"
#include <iostream>

namespace Eigen
{
    Matrix3d skew3(Eigen::Vector3d vec)
    {
        Eigen::Matrix3d mo; 
        mo << 0.0, -vec(2), vec(1),
             vec(2), 0.0, -vec(0), 
             -vec(1), vec(0), 0.0;
        return mo;
    }
}

namespace mathematic
{
    class quaternion
    {
        private:
        Eigen::Vector4d data; // wxyz
        public:
        quaternion(){}
        quaternion(double *_data) 
        {
            data = Eigen::Vector4d(_data);
        }
        quaternion(Eigen::Vector4d _data)
        {
            data = _data;
        }
        double x() {return data(1);}
        double y() {return data(2);}
        double z() {return data(3);}
        double w() {return data(0);}
        Eigen::Vector4d q() {return data;}
        Eigen::Matrix3d toRotationMatrix()
        {
            Eigen::Matrix<double, 3, 3> mo;
            double w = this->w();
            double x = this->x();
            double y = this->y();
            double z = this->z();
            mo << 1-2*y*y-2*z*z, 2*x*y-2*w*z, 2*x*z+2*w*y,
                 2*x*y+2*w*z,  1-2*x*x-2*z*z, 2*y*z-2*w*x,
                 2*x*z-2*w*y,  2*y*z+2*w*x,  1-2*x*x-2*y*y;
            return mo;
        }
        Eigen::Matrix4d skew()
        {
            Eigen::Matrix<double, 4, 4> mo;
            double w = this->w();
            double x = this->x();
            double y = this->y();
            double z = this->z();
            mo << w, -z, y, x, 
                z, w, -x, y, 
                -y, x, w, z,
                -x, -y, -z, w;
            return mo;
        }
        quaternion operator*(quaternion q)
        {
            Eigen::Vector4d mo = (this->skew() * q.data).transpose();
            return quaternion(mo);
        }
        quaternion operator+(quaternion q)
        {
            Eigen::Vector4d mo = data + q.data;
            return quaternion(mo);
        }
        quaternion operator-(quaternion q)
        {
            Eigen::Vector4d mo = data - q.data;
            return quaternion(mo);
        }
        void inverse()
        {
            data(0) = -data(0);
        }
        Eigen::Vector3d toEuler()
        {
            double w = this->w();
            double x = this->x();
            double y = this->y();
            double z = this->z();
            double sinr_cosp = 2 * (w * x + y * z);
            double cosr_cosp = 1 - 2 * (x * x + y * y);
            double roll = std::atan2(sinr_cosp, cosr_cosp);

            double sinp = std::sqrt(1 + 2 * (w * y - x * z));
            double cosp = std::sqrt(1 - 2 * (w * y - x * z));
            double pitch = 2 * std::atan2(sinp, cosp) - M_PI / 2;

            double siny_cosp = 2 * (w * z + x * y);
            double cosy_cosp = 1 - 2 * (y * y + z * z);
            double yaw = std::atan2(siny_cosp, cosy_cosp);
            return Eigen::Vector3d(roll, pitch, yaw);
        }
    };
    quaternion zeta(Eigen::Vector3d vec)
    {
        double norm2 = sqrt(vec.dot(vec));
        if (norm2 == 0) return quaternion(Eigen::Vector4d(1, 0, 0, 0));
        vec = sin(0.5 * norm2) * vec / norm2;
        double v[4] = {cos(0.5 * norm2), vec(0), vec(1), vec(2)};
        return quaternion(v);
    }
    int order(int n)
    {
        if (n == 0) return 1;
        else return n * order(n - 1);
    }
    template<int N>
    Eigen::Matrix3d Gamma(Eigen::Vector3d vec, double dt, int n)
    {
        int iter = N;
        if (N > 20) iter = 20;
        Eigen::Matrix3d g;
        Eigen::Matrix3d w_x = Eigen::skew3(vec);
        for (int i = 0; i < iter; i++)
        {
            if (i == 0) g = pow(dt, n) / (double) order(n) * Eigen::Matrix3d::Identity();
            else 
            {
                Eigen::Matrix3d w_x_n = w_x;
                for (int j = 1; j < i; j++) w_x_n = w_x_n * w_x;
                g = g + pow(dt, n+i) / (double) order(n+i) * w_x_n;
            }
        }
        return g;
    }
}

#endif