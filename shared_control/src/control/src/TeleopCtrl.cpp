#include "TeleopCtrl.h"

TeleopCtrl::TeleopCtrl(Eigen::Matrix<double, 4, 4> &T)
{
    T_ = T;
}

void TeleopCtrl::rateCtrl(const double p[3], const float k, Eigen::Matrix<double, 4, 4> &T)
{
    T_(0,3) += p[2]*k;
    T_(1,3) -= p[0]*k;
    T_(2,3) -= p[1]*k;

    T = T_;
}

void TeleopCtrl::posCtrl(const double p[3], const float k, Eigen::Matrix<double, 4, 4> &T)
{
    T_(0,3) = p[2]*k;
    T_(1,3) = -p[0]*k;
    T_(2,3) = -p[1]*k;

    T = T_;
}

void TeleopCtrl::posCtrlASME(const double p[3], const float kp, const float kv,  Eigen::Matrix<double, 4, 4> &T)
{
    T_(0,3) += p[2]*kv;
    T_(1,3) -= p[0]*kv;
    T_(2,3) = -p[1]*kp;

    T = T_;
}

void TeleopCtrl::posCtrlMSAE(const double p[3], const float kp, const float kv, Eigen::Matrix<double, 4, 4> &T)
{
    T_(0,3) = p[2]*kp;
    T_(1,3) = -p[0]*kp;
    T_(2,3) -= p[1]*kv;

    T = T_;
}

void TeleopCtrl::hapticGuidance(const Eigen::Matrix<float, 3, 1> &h_dir, const float h_int, std::array<double,3> &h_gui)
{
    h_gui[0] = -h_int*h_dir[1];
    h_gui[1] = -h_int*h_dir[2];
    h_gui[2] = h_int*h_dir[0];
}
