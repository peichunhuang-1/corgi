#ifndef LEG_HPP
#define LEG_HPP
#include "math.h"
#include "coordinate.hpp"
#include <Eigen/Dense>
namespace LinkLeg
{
    /* 
    This model is based on frame {L}, which oringinates from the joint connecting i-th leg to the body.
    And the x-axis is coincides with z-axis of body frame, y-axis is coincides with x-axis of body frame.
    */
    typedef enum RING_TYPE
    {
        RIGHT_UPPER_RING,
        RIGHT_DOWNWARD_RING,
        LEFT_UPPER_RING,
        LEFT_DOWNWARD_RING,
        G_POINT,
    }RING_TYPE;

    class LinkLeg
    {
        private:
            double _l1; // OA
            double _l2; // BD
            double _l3; // BE
            double _l4; // DE
            double _l5; // CD
            double _l6; // AB
            double _l7; // arc of BC
            double _l8; // arc of BF
            double _l9; // arc of BH
            double _l10; // arc of FG
            double _to1; // angle between BC and BO1 (at initial)
            double _to2; // angle between GF and GO2 (at initial)
            double _tf; // angle between BC and BF (at initial)
            double _th; // angle between BC and BH (at initial)

            double _R; // wheel radius
            double _min_theta;
            double _max_theta;

            double _oe; // distance between o and e
            double _oe_d; // differential of oe
            double _oe_dd; // differential of oe_d

            double _db; // distance between b and d
            double _db_d; // differential of db
            double _db_dd; // differential of db_d

            double _theta; // angle between x axis and OB
            double _theta_d; // differential of theta
            double _theta_dd; // differential of theta_d

            double _phi; // angle between x axis and EB
            double _phi_d; // differential of phi
            double _phi_dd; // differential of phi_d
            
            double _epsilon; // angle between x axis and DB
            double _epsilon_d; // differential of epsilon
            double _epsilon_dd; // differential of epsilon_d
            
            double _theta2; // angle between DB and BC
            double _theta2_d; // differential of theta2
            double _theta2_dd; // differential of theta2_d

            double _rho; // angle between x axis and GF
            double _rho_d; // differential of rho
            double _rho_dd; // differential of rho_d
            int _init = false; // if model was initialize for inverse
            double _max_r; // G.x at theta = 145
            double _min_r; // G.x at theta = 17
            void D_phi();
            void D_oe();
            void D_db();
            void D_epsilon();
            void D_theta2();
            void D_rho();
        public:
            LinkLeg() {}
            LinkLeg(double R, double min, double max) : _R(R), _min_theta(min), _max_theta(max)
            {
                _l1 = 0.8 * R; // OA
                _l2 = 0.9 * R; // BD
                _l3 = 1.3 * R; // BE
                _l4 = 0.4 * R; // DE
                _l5 = 0.88296634 * R; // CD
                _l6 = 0.2 * R; // AB
                _l7 = 2.0 * R * sin(M_PI * 101.0 / 360.0); // arc of BC
                _l8 = 2.0 * R * sin(M_PI * 113.0 / 360.0); // arc of BF
                _l9 = 2.0 * R * sin(M_PI * 17.0 / 360.0); // arc of BH
                _l10 = 2.0 * R * sin(M_PI * 50.0 / 360.0); // arc of FG
                _to1 = M_PI * 39.5 / 180.0; // angle between BC and BO1 (at initial)
                _to2 = - M_PI * 65.0 / 180.0; // angle between GF and GO2 (at initial)
                _tf = M_PI * 6.0 / 180.0; // angle between BC and BF (at initial)
                _th = M_PI * 121.0 / 180.0; // angle between BC and BH (at initial)
            }
            Cartesian A;
            Cartesian B;
            Cartesian C;
            Cartesian D;
            Cartesian E;
            Cartesian F;
            Cartesian G;
            Cartesian H;
            Cartesian O1;
            Cartesian O2;

            double phi() {return _phi;}
            double phi_d() {return _phi_d;}
            double phi_dd() {return _phi_dd;}
            
            double epsilon() {return _epsilon;}
            double epsilon_d() {return _epsilon_d;}
            double epsilon_dd() {return _epsilon_dd;}
            
            double theta2() { return _theta2;}
            double theta2_d() { return _theta2_d;}
            double theta2_dd() { return _theta2_dd;}

            double rho() { return _rho;}
            double rho_d() { return _rho_d;}
            double rho_dd() {return _rho_dd;}
            Cartesian symmetry(Cartesian P, int axis);
            void calculate(double theta, double theta_d, double theta_dd); // input in radius
            double inv(double r, RING_TYPE type); // return in degree
    };

    void LinkLeg::D_phi()
    {
        double ph = _l1 / _l3 * cos(_theta) / cos(_phi);
        _phi_d = ph * _theta_d;
        _phi_dd = ph *  _theta_dd 
                - _l1 / _l3 * (sin(_phi) * _theta_d * _theta_d
                + tan(_phi) * cos(_theta) * _theta_d * _phi_d) / cos(_phi);
    }

    void LinkLeg::D_oe()
    {
        _oe_d = -_l1 * sin(_theta) * _theta_d + _l3 * sin(_phi) * _phi_d;
        _oe_dd = -_l1 * sin(_theta) * _theta_dd + _l3 * sin(_phi) * _phi_dd 
                - _l1 * cos(_theta) * _theta_d * _theta_d + _l3 * cos(_phi) * _phi_d * _phi_d;
    }

    void LinkLeg::D_db()
    {
        double db2 = _l2 * _l6 * sin(M_PI - _theta + _phi);
        _db_d = db2 * (-_theta_d + _phi_d) / _db;
        _db_dd = db2  * (-_theta_dd + _phi_dd) / _db + 
                (-_theta_d + _phi_d) * (-_theta_d + _phi_d) / _db 
                - _db_d  * (-_theta_d + _phi_d) * db2 / _db / _db;
    }

    void LinkLeg::D_epsilon()
    {
        _epsilon_d = (_l2 * _l2 * _phi_d + _l6 * _l6 * _theta_d + _l2 * _l6 * cos(_phi - _theta) * (_phi_d - _theta_d)) / _db / _db;
        _epsilon_dd = (_l2 * _l2 * _phi_dd + _l6 * _l6 * _theta_dd + _l2 * _l6 * cos(_phi - _theta) * (_phi_dd - _theta_dd)
                        - _l2 * _l6 * sin(_phi - _theta) * (_phi * _phi - _theta * _theta)) / _db / _db
                        - 2 * (_l2 * _l2 * _phi_d + _l6 * _l6 * _theta_d + _l2 * _l6 * cos(_phi - _theta) * (_phi_d + _theta_d)) * _db_d / _db / _db / _db;
    }

    void LinkLeg::D_theta2()
    {
        _theta2_d = -1 / sin(_theta2) * (2 * _db_d * (_l5 * _l5 - _l7 * _l7)) / 4 / _db / _db / _l7;
        _theta2_dd = - (cos(_theta2) * _theta2_d) / sin(_theta2) / sin(_theta2) * (2 * _db_d * (_l5 * _l5 - _l7 *_l7)) / 4 / _db / _db / _l7
                    - 1 / sin(_theta2) * (2 * (_l5 * _l5 - _l7 * _l7) * _db_dd * (4 * _db* _db * _l7) - 16 * _db * _db_d * _db_d * _l7 * (_l5 * _l5 - _l7 * _l7)) / 16 / _l7 / _l7 / (_db * _db * _db * _db);
    }

    void LinkLeg::D_rho()
    {
        _rho_d = 1 / cos(_rho) * (_R * cos(_theta) * _theta_d + _l8 * cos(M_PI - _theta2 + _epsilon + _tf) * (-_theta2_d + _epsilon_d) ) / _l10;
        _rho_dd = (_R * _theta_dd * cos(_theta) - _R * _theta_d * _theta_d * sin(_theta) + _l8 * (-_theta2_dd + _epsilon_dd) * cos(M_PI - _theta2 + _epsilon + _tf) - _l8 * (-_theta2_d + _epsilon_d) * (-_theta2_d + _epsilon_d) * sin(M_PI - _theta2 + _epsilon + _tf)) / _l10 / cos(_rho) 
                + sin(_rho) * _rho_d * (_R * cos(_theta) * _theta_d + _l8 * cos(M_PI - _theta2 + _epsilon + _tf) * (-_theta2_d + _epsilon_d)) / _l10 / _l10 / cos(_rho) / cos(_rho);
    }

    void LinkLeg::calculate(double theta, double theta_d, double theta_dd)
    {
        _theta = theta;
        _theta_d = theta_d;
        _theta_dd = theta_dd;

        _phi = asin(_l1 * sin(_theta) / _l3);
        D_phi();
        A = Angular(_l1, _theta, M_PI_2).to_cartesian(2);
        B = Angular(_R, _theta, M_PI_2).to_cartesian(2);
        _oe = _l1 * cos(_theta) - _l3 * cos(_phi);
        E = Angular(_oe, 0, M_PI_2).to_cartesian(2);
        D_oe();
        D = E + Angular(_l4, _phi, M_PI_2).to_cartesian(2);
        double db_2 = _l2 * _l2 + _l6 * _l6 - 2 * _l2 * _l6 * cos(M_PI - _theta + _phi);
        _db = sqrt(db_2);
        D_db();
        _epsilon = atan2(_l2 * sin(_phi) + _l6 * sin(_theta), _l2 * cos(_phi) + _l6 * cos(_theta));
        D_epsilon();
        _theta2 = acos((db_2 + _l7 * _l7 - _l5 * _l5) / (2.0 * _db * _l7));
        D_theta2();
        double to = M_PI - _theta2 + _epsilon;
        C = B + Angular(_l7, to, M_PI_2).to_cartesian(2);
        O1 = B + Angular(_R, to + _to1, M_PI_2).to_cartesian(2);
        F = B + Angular(_l8, to + _tf, M_PI_2).to_cartesian(2);
        H = B + Angular(_l9, to + _th, M_PI_2).to_cartesian(2);
        _rho = asin((_R * sin(_theta) + _l8 * sin(to + _tf)) / _l10);
        D_rho();
        G = F - Angular(_l10, _rho, M_PI_2).to_cartesian(2);
        O2 = G + Angular(_R, _rho + _to2, M_PI_2).to_cartesian(2);
    }

    double LinkLeg::inv(double r, RING_TYPE type)
    {   
        if (! _init)
        {
            calculate(_min_theta / 180.0 * M_PI, 0, 0);
            _min_r = G.x();
            calculate(_max_theta / 180.0 * M_PI, 0, 0);
            _max_r = G.x();
            _init = true;
        }
        double theta_optimal = 17.0; 
        switch (type)
        {
            case RIGHT_UPPER_RING: // right upper ring

            break;
            case LEFT_UPPER_RING: // left upper ring

            break;
            case RIGHT_DOWNWARD_RING: // right downward ring

            break;
            case LEFT_DOWNWARD_RING: // left downward ring

            break;
            case G_POINT: // G 
                // inverse function was too complicated, use iteration to find inverse,
                // and wa can find that the function of G and theta almost linear in our working space (0, 145).
                // use linear function as initial guess. Since the function was simple, we can easily get its optimal direction.
                theta_optimal = (r - _min_r) / (_max_r - _min_r) * (_max_theta - _min_theta) + _min_theta;
                
                for (int i = 0; i < 10; i ++)
                {
                    this->calculate(theta_optimal * M_PI / 180.0, 0, 0);
                    theta_optimal -= 160.0 * (this->G.x() - r) / (_max_r - _min_r);
                }
            break;
        }
        return theta_optimal;
    }

    Cartesian LinkLeg::symmetry(Cartesian P, int axis)
    {
        Cartesian Po;
        if (axis == 0)
        {Po(-P.x(), P.y(), P.z());}
        else if(axis == 1)
        {Po(P.x(), -P.y(), P.z());}
        else if (axis == 2)
        {Po(P.x(), P.y(), -P.z());}
        else
        {Po(P.x(), P.y(), P.z());}
        return Po;
    }

    class Leg
    {
        private:
            LinkLeg link;
            Eigen::Matrix3d rot;
            /*
            0 1 0
            0 0 1
            1 0 0
            */
            
        public:
            Cartesian vec;
            Leg() {}
            Leg(double r, Eigen::Matrix3d R, Cartesian V) : link(r,  17.0 / 180.0 * M_PI, 160.0 / 180.0 * M_PI), rot(R), vec(V) {}
            Cartesian position(double theta, double phi)
            {
                link.calculate(theta, 0, 0);
                return link.G.rotate(phi, 0, 2).rotate(rot) + vec;
            }
            Eigen::Matrix<double, 3, 2> Jacobian(double theta, double phi)
            {
                double delta = 0.001;
                Cartesian right_point = this->position(theta + delta, phi);
                Cartesian left_point = this->position(theta - delta, phi);
                Cartesian front_point = this->position(theta, phi + delta);
                Cartesian back_point = this->position(theta, phi - delta);
                Cartesian delta_theta = right_point - left_point;
                Cartesian delta_phi = front_point - back_point;
                Eigen::Matrix<double, 3, 2> J;
                J << delta_theta.x(), delta_phi.x(), delta_theta.y(), delta_phi.y(), delta_theta.z(), delta_phi.z();
                J = J / delta * 0.5;
                return J;
            }
            std::pair<double, double> inv(Cartesian position)
            {
                position = position - vec;
                position = position.rotate(rot.transpose());
                std::pair<double, double> motor_angles;
                double r = sqrt(position.x() * position.x() + position.y() * position.y());
                motor_angles.first = link.inv(r, G_POINT); // theta: 開合角
                motor_angles.second = atan2(position.y(), position.x()); // beta: 腳的轉角
                return motor_angles;
            }
    };
    
}

#endif