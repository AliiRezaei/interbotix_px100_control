#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "interbotix_xs_msgs/JointGroupCommand.h"

// #include <string.h>


// char joints_name[4][12] = {"waist", "shoulder", "elbow", "wrist_angle"};

int main(int argc, char **argv)
{
    // std::string node_name = "px100_go_to_home";
    ros::init(argc, argv, "px100_go_to_home");
    ros::NodeHandle nh;

    // std::string topic_name = "/px100/joint_states";
    ros::Publisher joint_position_pub = nh.advertise<interbotix_xs_msgs::JointGroupCommand>("/px100/commands/joint_group", 1);
    ros::Rate rate(20);

    interbotix_xs_msgs::JointGroupCommand joint_state_msg;
    // joint_state_msg.name = "arm";
    // joint_state_msg.cmd = {0.0, 0.0, 0.0, 0.0, 0.0};
    // joint_position_pub.publish(joint_state_msg);

    while(ros::ok())
    {

        // joint_state_msg.name = {"waist", "shoulder", "elbow", "wrist_angle", "gripper", "left_finger", "right_finger"};

        joint_state_msg.name = "arm";
        joint_state_msg.cmd = {0.0, 0.0, 0.0, 0.0};


        joint_position_pub.publish(joint_state_msg);
        // ROS_INFO("published positions : %d", joint_state_msg.position);
        rate.sleep();
    }

    return 0;
}