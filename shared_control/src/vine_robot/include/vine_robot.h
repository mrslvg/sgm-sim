#ifndef VINE_ROBOT_H
#define VINE_ROBOT_H

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"


class vine_robot
{
public:

    vine_robot(float w_p_ee[3], const Eigen::Matrix4d &w_T_rb);
    void control(float w_p_ee_cmd[3], float *w_p_ee);
    void get_w_p_ee(float *w_p_ee);
    void get_rb_p_ee(float *rb_p_ee);

//    Eigen::Matrix<double,4,4> forward_kinematics(Eigen::Vector2d q);
//    Eigen::Matrix<double,2,2> jacobian(Eigen::Vector2d q);
//    Eigen::Vector2d getArcValues();
//    Eigen::Matrix4d get_w_rb() const;
//    void setArcValues(const Eigen::Vector2d &arc_values);
//    void set_w_rb(const Eigen::Matrix4d &w_rb);
//    void ik(const Eigen::Matrix4d &rb_ee, float &l, float &ax, float &ay);


private:

    void update(float w_p_ee[3]);
    float w_p_ee_[3] = {0,0,0}, rb_p_ee_[3] = {0,0,0};
    Eigen::Matrix4d w_T_rb_;
//    Eigen::Matrix4d m_w_rb;
//    Eigen::Vector2d m_arc_values;
//    Eigen::Vector2d m_l_lim;
//    Eigen::Vector2d m_k_lim;

};

#endif // VINE_ROBOT_H
