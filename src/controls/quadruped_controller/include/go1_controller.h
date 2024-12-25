#ifndef GO_CONTROLLER_H_
#define GO_CONTROLLER_H_
 
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <std_srvs/Empty.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <realtime_tools/realtime_publisher.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/ContactsState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Accel.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <cmath>
#include <thread>
#include <mutex>
#include "Eigen/Dense"
#include "go1_enum.h" 
#include "RobotLeg.h"
#include "TrajectoryGenerator_V2.h"    

#include <lcm/lcm-cpp.hpp>  
#include "state_estimator_lcmt.hpp"
#include "leg_control_data_lcmt.hpp"
#include "pd_tau_targets_lcmt.hpp"
#include "rc_command_lcmt.hpp" 

lcm::LCM lcm1;   

class go1_controller
{
public:
    go1_controller(ros::NodeHandle &nh,
                      std::string go1_topic_leg_state,
                      std::string go1_topic_leg_command,
                      std::string urdf_path,
                      std::vector<std::string> foot_name,
                      const double freq)
        : nh_(nh),
          go1_topic_leg_state_(go1_topic_leg_state),
          go1_topic_leg_command_(go1_topic_leg_command),
          freq_(freq),
          pi(M_PI),
          deg2rad(pi / 180),
          rad2deg(180 / pi)
    {}
    virtual ~go1_controller() {}

    void Init();
    void Run(); 
    void PolicyRun(); 

private:   
    /********************************************************************************************************************************/
    /* Quadruped variable */
    RobotLeg LEG[NUM_LEG];
    TrajectoryGenerator HOMING_TRAJ[NUM_LEG][NUM_JOINT]; 

    Eigen::VectorXd leg_q_, leg_q_dot_;  
    void SetLegJointAngle(Eigen::VectorXd q, Eigen::VectorXd q_dot);  

    int homing_flag_;
    Eigen::VectorXd homing_q_[NUM_LEG];
    int GetIsHoming() {return homing_flag_;}
    void JointHoming(); 
    
 
    // Function
    void StateBodyCallback(const gazebo_msgs::ModelStates::ConstPtr &body);
    void StateLegCallback(const sensor_msgs::JointState &state); 

    void ImuCallback(const sensor_msgs::Imu &msg);
    
    void Command(bool flag);
    void SendCommandsToRobot();
    void SendCommandsToRobot2();  
    void InitCount(); 

    // Variable
    ros::NodeHandle nh_;
    const double pi, deg2rad, rad2deg, freq_;
    const std::string go1_topic_leg_state_, go1_topic_leg_command_;
    ControlMode controlmode;

    bool Recieved_Joint_State;
    bool Recieved_Mode[NUM_MODE] = {false};
    double current_time_;
    int start_timing_print_[5];

    //  Subscribers:
    ros::Subscriber sub_body_states_; 
    ros::Subscriber sub_leg_state_;
    ros::Subscriber sub_imu_;

    //  Publisher:
    ros::Publisher pub_leg_cmd_;

    // Robot Parameter:
    Eigen::VectorXd q_, dq_, torque_;                   // 12 x 1

    // GAZEBO
    Eigen::VectorXd gazebo_body_pos, gazebo_body_vel;   // 3 x 1
    Eigen::VectorXd gazebo_quat;                        // 4 x 1
    Eigen::VectorXd gazebo_rpy, gazebo_rpy_dot;         // 3 x 1

    //Contact 
    Eigen::VectorXd contact_;                           // 4 x 1 
 
    int count_1khz;

    // IMU
    Eigen::Vector4d imu_quat, pino_quat;
    Eigen::Vector3d imu_rpy, imu_rpy_dot;
    Eigen::Matrix3d imu_rot; 
    Eigen::Vector3d imu_lin_acc; 

    Eigen::Matrix3d Quat2Rot(Eigen::Quaterniond q)
    {
        q.Eigen::Quaterniond::normalize();
        Eigen::Matrix3d rotation_matrix = q.Eigen::Quaterniond::toRotationMatrix();

        return rotation_matrix;
    }

    Eigen::Vector3d Quat2Euler(Eigen::Vector4d quat)
    { 
        double sinr_cosp = 2 * (quat(0) * quat(1) + quat(2) * quat(3));
        double cosr_cosp = 1 - 2 * (quat(1) * quat(1) + quat(2) * quat(2));
        double roll = std::atan2(sinr_cosp, cosr_cosp);

        double sinp = 2 * (quat(0) * quat(2) - quat(3) * quat(1));
        double pitch;
        if (std::abs(sinp) >= 1)
            pitch = std::copysign(M_PI / 2, sinp);
        else
            pitch = std::asin(sinp);

        double siny_cosp = 2 * (quat(0) * quat(3) + quat(1) * quat(2));
        double cosy_cosp = 1 - 2 * (quat(2) * quat(2) + quat(3) * quat(3));
        double yaw = std::atan2(siny_cosp, cosy_cosp);

        Eigen::Vector3d rpy(roll, pitch, yaw);
        return rpy;
    }
 
    Eigen::Quaterniond eigen_quat; 

    
    

protected:
};

#endif