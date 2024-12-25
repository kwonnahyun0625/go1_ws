#include "go1_controller.h"

double target_torque[12];

bool _firstCommandReceived;
bool _firstRun;
state_estimator_lcmt body_state_simple = {0};
leg_control_data_lcmt joint_state_simple = {0};
pd_tau_targets_lcmt joint_command_simple = {0};
rc_command_lcmt rc_command = {0};
int btn_y = 0;
int flag_target_torque = 0;

double lin_x = 0;
double lin_y = 0;
double lin_yaw = 0;

class LCMActionHandler
{
public:
    ~LCMActionHandler() {} 

    void handleActionLCM(const lcm::ReceiveBuffer *rbuf, const std::string & chan, const pd_tau_targets_lcmt * msg)
    {
        (void) rbuf;
        (void) chan;

        joint_command_simple = *msg; 
        _firstCommandReceived = true;
    }
};
LCMActionHandler action_handler_;
 


void go1_controller::Init()
{
    // Reset Gazebo Simualtion
    system("clear");
    std_srvs::Empty reset;
    ros::service::call("/gazebo/reset_simulation", reset);
    std::cout << "Reset Gazebo Simulation" << std::endl;
    nh_.ok();

    uint32_t queue_size = 10;
    // Subscribers
    sub_body_states_ = nh_.subscribe("/gazebo/model_states", queue_size, &go1_controller::StateBodyCallback, this, ros::TransportHints().reliable().tcpNoDelay());
    sub_leg_state_ = nh_.subscribe(go1_topic_leg_state_, queue_size, &
    go1_controller::StateLegCallback, this, ros::TransportHints().reliable().tcpNoDelay()); 
    sub_imu_ = nh_.subscribe("/imu/data", queue_size, &go1_controller::ImuCallback, this);

    // Publisher
    pub_leg_cmd_ = nh_.advertise<std_msgs::Float64MultiArray>(go1_topic_leg_command_, queue_size);

    controlmode = INIT;
    Recieved_Joint_State = false;

    current_time_ = 0.0;
    for (size_t i = 0; i < 5; i++)
    {
        start_timing_print_[i] = 0;
    }

    // Initial Size    
    q_.setZero(NUM_DOF);
    dq_.setZero(NUM_DOF);
    torque_.setZero(NUM_DOF);  

    gazebo_body_pos.setZero(NUM_AXIS); 
    gazebo_body_vel.setZero(NUM_AXIS);
    gazebo_quat.setZero(4);
    gazebo_rpy.setZero(NUM_AXIS); 
    gazebo_rpy_dot.setZero(NUM_AXIS);

    contact_.setZero(NUM_LEG);
    count_1khz = 0;

    /***********************************************/
    /* Quadruped Homing */
    for (size_t i = 0; i < NUM_LEG; i++)
    {
        homing_q_[i].setZero(NUM_JOINT);
    }
    homing_flag_ = false;
    /***********************************************/ 
}
 


void go1_controller::SendCommandsToRobot()
{
    std_msgs::Float64MultiArray msg;

    for (size_t i = 0; i < NUM_DOF; i++)
    {
        if (controlmode == INIT)
        {
            msg.data.push_back(0);
        }
        else
        {
            msg.data.push_back(torque_(i));
        }
    }

    pub_leg_cmd_.publish(msg);

    msg.data.clear();
}

void go1_controller::SendCommandsToRobot2()
{
    std_msgs::Float64MultiArray msg;

    for (size_t i = 0; i < NUM_DOF; i++)
    {
        if (controlmode == INIT)
        {
            msg.data.push_back(0);
        }
        else
        {
            msg.data.push_back(target_torque[i]);
        }
    }

    pub_leg_cmd_.publish(msg);

    msg.data.clear();
}


void go1_controller::StateBodyCallback(const gazebo_msgs::ModelStates::ConstPtr &body)
{
    std::vector<std::string>::const_iterator iter = std::find(body->name.begin(), body->name.end(), "go1");
    if (iter != body->name.end())
    {
        int index = iter - body->name.begin();

        geometry_msgs::Point position = body->pose[index].position;           // position
        geometry_msgs::Quaternion orientation = body->pose[index].orientation;
        geometry_msgs::Vector3 velocity = body->twist[index].linear;          // velocity
        geometry_msgs::Vector3 angular_velocity = body->twist[index].angular; // angular velocity

        tf::Quaternion q(
            body->pose[index].orientation.x,
            body->pose[index].orientation.y,
            body->pose[index].orientation.z,
            body->pose[index].orientation.w
        );
        eigen_quat = Eigen::Quaterniond(q[3], q[0], q[1], q[2]);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        gazebo_body_pos(X) = position.x;
        gazebo_body_pos(Y) = position.y;
        gazebo_body_pos(Z) = position.z;

        gazebo_body_vel(X) = velocity.x;
        gazebo_body_vel(Y) = velocity.y;
        gazebo_body_vel(Z) = velocity.z;

        gazebo_quat(0) = orientation.w;
        gazebo_quat(1) = orientation.x;
        gazebo_quat(2) = orientation.y;
        gazebo_quat(3) = orientation.z;

        gazebo_rpy(0) = roll;
        gazebo_rpy(1) = pitch;
        gazebo_rpy(2) = yaw;

        gazebo_rpy_dot(0) = angular_velocity.x;
        gazebo_rpy_dot(1) = angular_velocity.y;
        gazebo_rpy_dot(2) = angular_velocity.z;
    }
}

void go1_controller::StateLegCallback(const sensor_msgs::JointState &state)
{
    // HY, HP, HR, KR
    for (size_t i = 0; i < 12; i++)
    {
        q_(i) = state.position[i];
        dq_(i) = state.velocity[i];

        Recieved_Joint_State = true;
    }  
}


void go1_controller::InitCount()
{
    current_time_ += 0.005;
    if (current_time_ >= 5)
    {
        std::cout << "5 .....\n";
        controlmode = HOMING;
    }
    else if (current_time_ >= 4 && start_timing_print_[4] == 0)
    {
        std::cout << "4 ....\n";
        start_timing_print_[4] = 1;
    }
    else if (current_time_ >= 3 && start_timing_print_[3] == 0)
    {
        std::cout << "3 ...\n";
        start_timing_print_[3] = 1;
    }
    else if (current_time_ >= 2 && start_timing_print_[2] == 0)
    {
        std::cout << "2 ..\n";
        start_timing_print_[2] = 1;
    }
    else if (current_time_ >= 1 && start_timing_print_[1] == 0)
    {
        std::cout << "1 .\n";
        start_timing_print_[1] = 1;
    }
    else if (start_timing_print_[0] == 0)
    {
        std::cout << "0 \n";
        start_timing_print_[0] = 1;
    }
}
 
void go1_controller::SetLegJointAngle(Eigen::VectorXd q, Eigen::VectorXd q_dot)
{
    leg_q_ = q;
    leg_q_dot_ = q_dot;
    for (size_t i = 0; i < NUM_LEG; i++)
    {
        LEG[i].SetJointAngle(leg_q_.block(i * NUM_JOINT, 0, NUM_JOINT, 1), leg_q_dot_.block(i * NUM_JOINT, 0, NUM_JOINT, 1));
    }
}  
void go1_controller::ImuCallback(const sensor_msgs::Imu &msg)
{ 
    Eigen::Quaterniond quat(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
    pino_quat << quat.x(), quat.y(), quat.z(), quat.w();
    imu_quat << quat.w(), quat.x(), quat.y(), quat.z();

    imu_rot = Quat2Rot(quat);

    imu_rpy = Quat2Euler(imu_quat);

    imu_rpy_dot << msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z;

    imu_lin_acc << msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z;
 
    Recieved_Joint_State = true;
}

void go1_controller::JointHoming()
{
    if (HOMING_TRAJ[0][0].GetState() == 0)
    {
        for (size_t i = 0; i < NUM_LEG; i++)
        {
            homing_q_[i] << 0, 0.67, -1.3;
            for (size_t j = 0; j < NUM_JOINT; j++)
            {
                HOMING_TRAJ[i][j].SetInitialPosition(leg_q_(i * NUM_JOINT + j));
                HOMING_TRAJ[i][j].SetBasicParameters(homing_q_[i](j), 3);
            }
        } 
    }
    else if (HOMING_TRAJ[0][0].GetState() == 1)
    {
        homing_flag_ = true;
    }

    Eigen::VectorXd q_d[NUM_LEG], q_dot_d[NUM_LEG];
    for (size_t i = 0; i < NUM_LEG; i++)
    {
        q_d[i].setZero(NUM_JOINT); 
        q_dot_d[i].setZero(NUM_JOINT);
        for (size_t j = 0; j < NUM_JOINT; j++)
        {
            HOMING_TRAJ[i][j].ComputeTrajectory();
            q_d[i](j) = HOMING_TRAJ[i][j].GetPosition();
            q_dot_d[i](j) = HOMING_TRAJ[i][j].GetVelocity();
        }
        LEG[i].JointSpacePD(q_d[i], q_dot_d[i]);
        for (size_t j = 0; j < NUM_JOINT; j++)
        {
            torque_(i * NUM_JOINT + j) = LEG[i].GetTargetTorque()(j);
        }
    }
    
}
 
void go1_controller::Run()
{
    ROS_INFO("Running the torque control loop .................");
      
    lcm1.subscribe("pd_plustau_targets", &LCMActionHandler::handleActionLCM, &action_handler_);
    _firstCommandReceived = false;
    _firstRun = true;

    const ros::Duration control_period_(1 / freq_);

    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::Time start_time = ros::Time::now();
    ros::Time last_control_time = start_time;

    while (ros::ok())
    {
        ros::Time current_time = ros::Time::now();

        ros::Duration elapsed_time = current_time - last_control_time;

        if (elapsed_time >= control_period_)
        {
            // UpdatDiffe the last control time
            last_control_time = current_time;

            // Perform control actions here
            // ROS_INFO("Control loop running");
            Command(Recieved_Joint_State);

            // Sleep to enforce the desired control loop frequency
            ros::Duration sleep_time = control_period_ - elapsed_time;
            if (sleep_time > ros::Duration(0))
            {
                sleep_time.sleep();
            }
        }
    }
}

void go1_controller::PolicyRun()
{
    ROS_INFO("Running the lcm control loop .................");

    const ros::Duration control_period_(1.0 / 50);

    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::Time start_time = ros::Time::now();
    ros::Time last_control_time = start_time;

    while (ros::ok())
    {
        ros::Time current_time = ros::Time::now();

        ros::Duration elapsed_time = current_time - last_control_time;

        if (elapsed_time >= control_period_)
        {
            last_control_time = current_time;

            lcm1.handle();
            flag_target_torque = 1;

            ros::Duration sleep_time = control_period_ - elapsed_time;
            if (sleep_time > ros::Duration(0))
            {
                sleep_time.sleep();
            }
        }
    }
} 
 
void go1_controller::Command(bool flag)
{
    if (flag)
    {   

        SetLegJointAngle(q_, dq_);

        switch (controlmode)
        {
        case INIT:
        {
            if (!Recieved_Mode[INIT])
            {
                std::cout << "INIT MODE" << std::endl;
                Recieved_Mode[INIT] = true;
                Recieved_Mode[HOMING] = false;
                Recieved_Mode[RUNNING] = false; 
            }
            InitCount();
            break;
        }
        case HOMING:
        {
            if (!Recieved_Mode[HOMING])
            {
                std::cout << "HOMING MODE" << std::endl;

                Recieved_Mode[INIT] = false;
                Recieved_Mode[HOMING] = true;
                Recieved_Mode[RUNNING] = false; 
            }

            if (GetIsHoming() == false)
            {
                JointHoming();
            }
            else if (GetIsHoming() == true)
            { 
                controlmode = RUNNING;
            }
            SendCommandsToRobot();
            break;
        }
        case RUNNING:
        {
            if (!Recieved_Mode[RUNNING])
            {
                std::cout << "RUNNING_WM MODE" << std::endl;

                Recieved_Mode[INIT] = false;
                Recieved_Mode[HOMING] = false;
                Recieved_Mode[RUNNING] = true; 
                 
            } 

            
            
            if (count_1khz % 1 == 0)
            { 
                rc_command.left_stick[0] = lin_y;
                rc_command.left_stick[1] = lin_x;
                rc_command.right_stick[0] = (-1) * lin_y;
                rc_command.right_stick[1] = lin_y;
                rc_command.right_lower_right_switch = btn_y;
                rc_command.right_upper_switch = 0;
                rc_command.left_lower_left_switch = 0;
                rc_command.left_upper_switch = 0;
                rc_command.mode = 0; 

                joint_state_simple.q[0] = q_[3];
                joint_state_simple.q[1] = q_[4];
                joint_state_simple.q[2] = q_[5]; 
                joint_state_simple.q[3] = q_[0];
                joint_state_simple.q[4] = q_[1];
                joint_state_simple.q[5] = q_[2]; 
                joint_state_simple.q[6] = q_[9];
                joint_state_simple.q[7] = q_[10];
                joint_state_simple.q[8] = q_[11]; 
                joint_state_simple.q[9] = q_[6];
                joint_state_simple.q[10] = q_[7];
                joint_state_simple.q[11] = q_[8];

                joint_state_simple.qd[0] = dq_[3];
                joint_state_simple.qd[1] = dq_[4];
                joint_state_simple.qd[2] = dq_[5]; 
                joint_state_simple.qd[3] = dq_[0];
                joint_state_simple.qd[4] = dq_[1];
                joint_state_simple.qd[5] = dq_[2]; 
                joint_state_simple.qd[6] = dq_[9];
                joint_state_simple.qd[7] = dq_[10];
                joint_state_simple.qd[8] = dq_[11]; 
                joint_state_simple.qd[9] = dq_[6];
                joint_state_simple.qd[10] = dq_[7];
                joint_state_simple.qd[11] = dq_[8];

                joint_state_simple.tau_est[0] = torque_[3];
                joint_state_simple.tau_est[1] = torque_[4];
                joint_state_simple.tau_est[2] = torque_[5]; 
                joint_state_simple.tau_est[3] = torque_[0];
                joint_state_simple.tau_est[4] = torque_[1];
                joint_state_simple.tau_est[5] = torque_[2]; 
                joint_state_simple.tau_est[6] = torque_[9];
                joint_state_simple.tau_est[7] = torque_[10];
                joint_state_simple.tau_est[8] = torque_[11]; 
                joint_state_simple.tau_est[9] = torque_[6];
                joint_state_simple.tau_est[10] = torque_[7];
                joint_state_simple.tau_est[11] = torque_[8];


                for(int i = 0; i < 4; i++){ 
                    body_state_simple.quat[i] = gazebo_quat[i];
                    
                } 

                for(int i = 0; i < 3; i++){
                    body_state_simple.rpy[i] = gazebo_rpy[i]; 
                    body_state_simple.aBody[i] = imu_lin_acc[i];
                    body_state_simple.omegaBody[i] = imu_rpy_dot[i];
                }
                for(int i = 0; i < 4; i++){
                    body_state_simple.contact_estimate[i] = contact_[i];
                }

                lcm1.publish("state_estimator_data", &body_state_simple);
                lcm1.publish("leg_control_data", &joint_state_simple);
                lcm1.publish("rc_command", &rc_command);

                

                if(_firstRun && joint_state_simple.q[0] != 0){
                    for(int i = 0; i < 12; i++){
                        joint_command_simple.q_des[i] = joint_state_simple.q[i];
                    }
                    _firstRun = false;
                    
                } 
                
            } 

            if (flag_target_torque == 0)
            {
                for (int i = 0; i < 12; i++)
                {
                    double Kp = 20;
                    double Kd = 0.5;
                    
                    target_torque[i] = Kp*(joint_command_simple.q_des[i]-q_[i]) - Kd*dq_[i]; 
                }
                SendCommandsToRobot2();
            }
            else
            {
                double Kp = 20;
                double Kd = 0.5;
                    
                target_torque[0] = Kp*(joint_command_simple.q_des[3]-q_[0]) - Kd*dq_[0]; 
                target_torque[1] = Kp*(joint_command_simple.q_des[4]-q_[1]) - Kd*dq_[1]; 
                target_torque[2] = Kp*(joint_command_simple.q_des[5]-q_[2]) - Kd*dq_[2]; 

                target_torque[3] = Kp*(joint_command_simple.q_des[0]-q_[3]) - Kd*dq_[3]; 
                target_torque[4] = Kp*(joint_command_simple.q_des[1]-q_[4]) - Kd*dq_[4]; 
                target_torque[5] = Kp*(joint_command_simple.q_des[2]-q_[5]) - Kd*dq_[5]; 

                target_torque[6] = Kp*(joint_command_simple.q_des[9]-q_[6]) - Kd*dq_[6]; 
                target_torque[7] = Kp*(joint_command_simple.q_des[10]-q_[7]) - Kd*dq_[7]; 
                target_torque[8] = Kp*(joint_command_simple.q_des[11]-q_[8]) - Kd*dq_[8]; 


                target_torque[9] = Kp*(joint_command_simple.q_des[6]-q_[9]) - Kd*dq_[9]; 
                target_torque[10] = Kp*(joint_command_simple.q_des[7]-q_[10]) - Kd*dq_[10]; 
                target_torque[11] = Kp*(joint_command_simple.q_des[8]-q_[11]) - Kd*dq_[11];  

                SendCommandsToRobot2();
            }
            
            
            count_1khz ++;
            break;
        }

        }
        
    }
    else
    { 
    }
}