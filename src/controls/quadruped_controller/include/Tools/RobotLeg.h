#ifndef ROBOTLEG_H_
#define ROBOTLEG_H_

#include <iostream>
#include <string>
#include <cmath>
#include "Eigen/Dense"
#include "go1_enum.h"
#include "NumericalTool.h"

class RobotLeg
{
public:
    RobotLeg();
    ~RobotLeg();

    /* Input */
    void SetJointAngle(Eigen::VectorXd q, Eigen::VectorXd q_dot); // 실제로봇state받는함수
    
    
    /* Output */
    Eigen::VectorXd GetTargetTorque() { return torque_; } // 토크내보내는 함수

    /* Function */
    void JointSpacePD(Eigen::VectorXd q_d, Eigen::VectorXd q_dot_d);// joint PD 

private:
    /* Object */
    // None
    NumericalTool::Calculus q_dot_calculus_[NUM_AXIS];
    // NumericalTool::LowPassFilter q_ddot_LPF_[NUM_AXIS];

    /* Input */
    Eigen::VectorXd q_, q_dot_, q_ddot_;        //actual joint angle        // 3 x 1
    Eigen::VectorXd q_d_, q_dot_d_, q_ddot_d_;        //desired joint angle        // 3 x 1
    
    
    /* Output */
    Eigen::VectorXd torque_;                    // 토크                      // 3 x 1

    /* Function */

};

#endif