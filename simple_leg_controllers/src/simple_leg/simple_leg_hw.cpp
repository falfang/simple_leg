/**
 * @file simple_leg_hw.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-10-05
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <simple_leg_controllers/simple_leg_hw.h>

namespace simple_leg{
// Constructor
simple_leg::simple_leg()
{   
    jnt_pos_.resize(n_dof_);
    jnt_vel_.resize(n_dof_);
    jnt_eff_.resize(n_dof_);
    jnt_pos_cmd_.resize(n_dof_);
    
    // connect with joint_state_interface
    hardware_interface::JointStateHandle hip_state_handle("Hip", &jnt_pos_[0], &jnt_vel_[0], &jnt_eff_[0]);
    joint_state_interface.registerHandle(hip_state_handle);
    hardware_interface::JointStateHandle knee_state_handle("Knee", &jnt_pos_[1], &jnt_vel_[1], &jnt_eff_[1]);
    joint_state_interface.registerHandle(knee_state_handle);
    registerInterface(&joint_state_interface);

    // connect with leg_position_interface
    hardware_interface::LegPositionHandle leg_position_handle("FL", &cmd_x, &cmd_z); // getHandle()と名前を合わせること！
    leg_position_interface.registerHandle(leg_position_handle);
    registerInterface(&leg_position_interface);

    hip_cmd_pub  = nh.advertise<std_msgs::Float64>("hip_command", 1);
    knee_cmd_pub = nh.advertise<std_msgs::Float64>("knee_command", 1);
}

// calculate Inverse Kinematics for designated position ("cmd_x" & "cmd_z" variable)
void simple_leg::solve_IK(void){
    double L = std::hypot(cmd_x, cmd_z);

    hip_cmd.data  = -atan2(cmd_x, -cmd_z) + acos((pow(L, 2) + pow(l1, 2) - pow(l2, 2)) / (2 * L * l1));
    knee_cmd.data = -M_PI + acos((pow(l1, 2.0) + pow(l2, 2.0) - pow(L, 2)) / (2 * l1 * l2));
}

// 実機ロボットから応答値を受け取る
void simple_leg::read(ros::Time time, ros::Duration period){
    jnt_pos_[0] = 1.0;
    jnt_pos_[1] = 2.0;
    // 今回は逆運動学計算で求めた結果をjoint_stateで返す
}


// 実機ロボットに指令値を送る
void simple_leg::write(ros::Time time, ros::Duration period){
    solve_IK();
    hip_cmd_pub.publish(hip_cmd);
    knee_cmd_pub.publish(knee_cmd);
}

}