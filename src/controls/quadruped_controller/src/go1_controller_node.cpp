#include "go1_controller.h"

void MainThreadRun(go1_controller &controller)
{
    controller.Init();
    controller.Run();
}
void PolicyThreadRun(go1_controller &controller)
{
    controller.PolicyRun();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "go1_node");
    ros::NodeHandle nh;

    // Variable
    const double freq = HZ_CONTROL; 
    std::string go1_urdf_path_ = "/home/kwon/nahyun/go1_ws/src/robots/go1_description/go1/urdf/go1.urdf";
    std::vector<std::string> go1_foot_name_ = {"F1_Foot", "F2_Foot", "H1_Foot", "H2_Foot"};

    std::string go1_topic_leg_state;
    std::string go1_topic_leg_command;

    // Topic names
    if (!nh.getParam("go1_topic_leg_state", go1_topic_leg_state))
    {
        ROS_ERROR("Couldn't retrieve the topic name for the state of the leg.");
        return -1;
    }
    if (!nh.getParam("go1_topic_leg_command", go1_topic_leg_command))
    {
        ROS_ERROR("Couldn't retrieve the topic name for commanding the  leg.");
        return -1;
    }

    // stquad_controller class
    go1_controller go1_controller(nh, go1_topic_leg_state, go1_topic_leg_command, go1_urdf_path_, go1_foot_name_, freq);

    std::thread main_thread(MainThreadRun, std::ref(go1_controller)); 
    std::thread policy_thread(PolicyThreadRun, std::ref(go1_controller));

    // Controller Start
    main_thread.join(); 
    policy_thread.join();

    return 0;
}