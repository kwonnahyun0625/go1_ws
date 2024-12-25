#include "RobotLeg.h"

RobotLeg::RobotLeg()
{
    q_.setZero(NUM_JOINT);
    q_dot_.setZero(NUM_JOINT);
    q_ddot_.setZero(NUM_JOINT);
    q_d_.setZero(NUM_JOINT);
    q_dot_d_.setZero(NUM_JOINT);
    q_dot_d_.setZero(NUM_JOINT); 
    torque_.setZero(NUM_JOINT);
}

RobotLeg::~RobotLeg()
{
}

void RobotLeg::SetJointAngle(Eigen::VectorXd q, Eigen::VectorXd q_dot)
{
    q_ = q;
    q_dot_ = q_dot;
    for (size_t i = 0; i < NUM_JOINT; i++)
    {
        q_ddot_(i) = q_dot_calculus_[i].Diff(q_dot_(i)); 
    }
} 
void RobotLeg::JointSpacePD(Eigen::VectorXd q_d, Eigen::VectorXd q_dot_d)
{
    Eigen::Vector3d Kp, Kd;
    Kp << Eigen::Vector3d::Ones() * 100;
    Kd << Eigen::Vector3d::Ones() * 1;

    for (size_t i = 0; i < NUM_JOINT; i++)
    {
        q_d_(i) = q_d(i);
        q_dot_d_(i) = q_dot_d(i);
    }

    for (size_t i = 0; i < NUM_JOINT; i++)
    {
        torque_(i) = Kp(i) * (q_d_(i) - q_(i)) + Kd(i) * (q_dot_d_(i) - q_dot_(i));
    }
} 