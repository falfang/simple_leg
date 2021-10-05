#include <ros/package.h>
#include <angles/angles.h>
#include <ros_control_tutorial/realhw.h>
#include <iostream>
#include <cmath>

TRobo::TRobo(){
    // connect and resister the joint state interface
    hardware_interface::JointStateHandle state_handle_1("joint1", &pos_[0], &vel_[0], &eff_[0]);
    joint_state_interface.registerHandle(state_handle_1);
    registerInterface(&joint_state_interface);

    // connect and register the joint position interface
    hardware_interface::JointHandle position_handle_1(joint_state_interface.getHandle("joint1"), &cmd_[0]);
    joint_position_interface.registerHandle(position_handle_1);
    registerInterface(&joint_position_interface);
}



void TRobo::timeCallback(const std_msgs::Float32::ConstPtr& msg_sub){
    pos_[0] = msg_sub -> data;  // set pos_[0] as real_hw date
}


void TRobo::read(ros::Time time, ros::Duration period){
    sub = nh.subscribe("response", 100000, &TRobo::timeCallback, this); // to use subscriber
}

// 今回は位置制御なので位置情報のみをros_controlに渡す
void TRobo::write(ros::Time time, ros::Duration period){
    std_msgs::Float32 msg_pub;
    msg_pub.data = cmd_[0];
    pub = nh.advertise<std_msgs::Float32>("request", 100000, true);
    pub.publish(msg_pub);
}