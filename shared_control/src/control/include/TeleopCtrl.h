#ifndef TELEOP_CTRL_H
#define TELEOP_CTRL_H

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

class TeleopCtrl
{
public:

    TeleopCtrl(Eigen::Matrix<double,4,4> &T);
    void rateCtrl(const double p[3], const float k, Eigen::Matrix<double, 4, 4> &T);
    void posCtrl(const double p[3], const float k, Eigen::Matrix<double, 4, 4> &T);
    void posCtrlASME(const double p[3], const float kp, const float kv, Eigen::Matrix<double, 4, 4> &T);
    void posCtrlMSAE(const double p[3], const float kp, const float kv, Eigen::Matrix<double, 4, 4> &T);
    void hapticGuidance(const Eigen::Matrix<float, 3, 1> &h_dir, const float h_int, std::array<double,3> &h_gui);

private:

    Eigen::Matrix<double,4,4> T_;

};

#endif // TELEOP_CTRL_H
