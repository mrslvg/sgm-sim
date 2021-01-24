#include "vine_robot.h"

vine_robot::vine_robot(float w_p_ee[3], const Eigen::Matrix4d& w_T_rb)
{
    for(unsigned int i = 0; i < 3; i ++)
        w_p_ee_[i] = w_p_ee[i];

    w_T_rb_ = w_T_rb;
    update(w_p_ee);
}

void vine_robot::update(float w_p_ee[3])
{

    for(unsigned int i = 0; i < 3; i ++)
    {
        rb_p_ee_[i] = w_T_rb_(0,i)*(w_p_ee[0]-w_T_rb_(0,3))+w_T_rb_(1,i)*(w_p_ee[1]-w_T_rb_(1,3))+w_T_rb_(2,i)*(w_p_ee[2] - w_T_rb_(2,3));
        w_p_ee_[i] = w_p_ee[i];
    }
}

void vine_robot::control(float w_p_ee_cmd[3], float *w_p_ee)
{
    for(unsigned int i = 0; i < 3; i ++)
        w_p_ee[i] = 0.01*w_p_ee_cmd[i] + 0.99*w_p_ee_[i];
    update(w_p_ee);
}

void vine_robot::get_w_p_ee(float *w_p_ee)
{
    for(unsigned int i = 0; i < 3; i ++)
        w_p_ee[i] = w_p_ee_[i];
}

void vine_robot::get_rb_p_ee(float *rb_p_ee)
{
    for(unsigned int i = 0; i < 3; i ++)
        rb_p_ee[i] = rb_p_ee_[i];
}


//void vine_robot::ik(const Eigen::Matrix4d &rb_ee, float& l, float& ax, float& ay)
//{
//    l = sqrt(pow(rb_ee(0,3),2) + pow(rb_ee(1,3),2) + pow(rb_ee(2,3),2));
//    ax = atan2(rb_ee(1,3), rb_ee(2,3));
//    ay = atan2(rb_ee(0,3), rb_ee(2,3));
//}

//Eigen::Vector2d vine_robot::getArcValues()
//{
//    return m_arc_values;
//}

//void vine_robot::setArcValues(const Eigen::Vector2d &arc_values)
//{
//    m_arc_values = arc_values;
//}

//Eigen::Matrix4d vine_robot::get_w_rb() const
//{
//    return m_w_rb;
//}

//void vine_robot::set_w_rb(const Eigen::Matrix4d &w_rb)
//{
//    m_w_rb = w_rb;
//}

//Eigen::Matrix<double,4,4> vine_robot::forward_kinematics(Eigen::Vector2d q) // planar constant curvature
//{
//    double k1 = q(0);
//    double l1 = q(1);

//    m_arc_values = q;

//    Eigen::Matrix<double,4,4> A0 = Eigen::Matrix<double,4,4>::Identity();
//    double t3 = k1*l1*(1.0/2.0);
//    double t2 = sin(t3);
//    double t4 = cos(t3);
//    double t5 = t2*t2;
//    double t6 = t4*t4;
//    double t7 = -t5+t6;
//    double t8 = 1.0/k1;
//    A0(0,0) = t7;
//    A0(0,1) = t2*t4*-2.0;
//    A0(0,3) = t2*t4*t8*2.0;
//    A0(1,0) = t2*t4*2.0;
//    A0(1,1) = t7;
//    A0(1,3) = t5*t8*2.0;
//    A0(2,2) = 1.0;
//    A0(3,3) = 1.0;

//    return A0;
//}

//Eigen::Matrix<double,2,2> vine_robot::jacobian(Eigen::Vector2d q) // planar constant curvature
//{
//    double k1 = q(0);
//    double l1 = q(1);

//    Eigen::Matrix<double,2,2> A0 = Eigen::Matrix<double,2,2>::Zero();

//    double t2 = k1*l1;
//    double t3 = cos(t2);
//    double t4 = 1.0/(k1*k1);
//    double t5 = sin(t2);
//    A0(0,0) = -t4*(t5-k1*l1*t3);
//    A0(0,1) = t3;
//    A0(1,0) = t4*(t3+k1*l1*t5-1.0);
//    A0(1,1) = t5;
//    A0(2,0) = l1;
//    A0(2,1) = k1;

//    return A0;
//}
