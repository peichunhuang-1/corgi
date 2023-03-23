#ifndef COORDINATE_HPP
#define COORDINATE_HPP
#include <cmath>
#include <Eigen/Dense>
// Two expression of Coordinates, spherical and cartesian.
// functions including exchange between two coordinates, initializing, rotate and basic operators.

class Angular;

class Cartesian
{
    private:
        /* data */
        Eigen::Vector3d data; // 1 x 3 (x, y, z)
    public:
        Cartesian() {}
        Cartesian(Eigen::Vector3d _data) {data = _data;}
        Cartesian(double x, double y, double z)
        {
            data(0) = x;
            data(1) = y;
            data(2) = z;
        }
        double x() {return data(0);}
        double y() {return data(1);}
        double z() {return data(2);}

        Cartesian operator+ (const Cartesian c) const;
        Cartesian operator- (const Cartesian c) const;
        void operator()(double x, double y, double z);
        void operator()(Eigen::Vector3d _data);

        Cartesian rotate(double beta, double gamma, int dimension);
        Cartesian rotate(Eigen::Matrix3d rot);
        Angular to_angular(int dimension);
};

class Angular
{
    private:
        double _r;
        double _theta;
        double _phi;
    public:
        Angular();
        Angular(double r, double theta, double phi) : _r(r), _theta(theta), _phi(phi) {}
        double r(){return _r;}
        double theta(){return _theta;}
        double phi(){return _phi;}
        Angular rotate(double beta, double gamma, int dimension);
        Cartesian to_cartesian(int dimension);
};

Angular Cartesian::to_angular(int dimension)
{
    double x = this->x();
    double y = this->y();
    double z = this->z();
    if (dimension == 2)
    {
        double r = sqrt(x * x + y * y);
        double theta = atan2(y, x);
        return Angular(r, theta, 0);
    }
    else if (dimension == 3)
    {
        double xxyy = x * x + y * y;
        double r = sqrt(xxyy + z * z);
        double theta = atan2(y, x);
        double phi = atan2(sqrt(xxyy), z);
        return Angular(r, theta, phi);
    }
    else
    {
        return Angular(0, 0, 0);
    }
}

Cartesian Cartesian::operator+ (const Cartesian c) const
{
    Cartesian co(data + c.data);
    return co;
}

Cartesian Cartesian::operator- (const Cartesian c) const
{
    Cartesian co(data - c.data);
    return co;
}

void Cartesian::operator() (double x, double y, double z)
{
    data(0) = x;
    data(1) = y;
    data(2) = z;
}

void Cartesian::operator()(Eigen::Vector3d _data)
{
    data = _data;
}

Cartesian Cartesian::rotate(double beta, double gamma, int dimension)
{
    Cartesian ro(data);
    return ro.to_angular(dimension).rotate(beta, gamma, dimension).to_cartesian(dimension);
}

Cartesian Cartesian::rotate(Eigen::Matrix3d rot)
{
    Eigen::Vector3d vo = rot * data;
    return Cartesian(vo);
}

Cartesian Angular::to_cartesian(int dimension)
{
    if (dimension == 2)
    {
        double x = _r * cos(_theta);
        double y = _r * sin(_theta);
        return Cartesian(x, y, 0);
    }
    else if (dimension == 3)
    {
        double rr = _r * sin(_phi);
        double x = rr * cos(_theta);
        double y = rr * sin(_theta);
        double z = _r * cos(_phi);
        return Cartesian(x, y, z);
    }
    else
    {
        return Cartesian(0, 0, 0);
    }
}

Angular Angular::rotate(double beta, double gamma, int dimension)
{
    if (dimension == 3)
        return Angular(_r, _theta + beta, _phi + gamma);
    else if (dimension == 2)
        return Angular(_r, _theta + beta, _phi);
    else
        return Angular(_r, _theta, _phi);
}
#endif