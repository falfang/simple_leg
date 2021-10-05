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
#include <ros_control_tutorial/simple_leg_hw.h>

namespace simple_leg{
// Constructor
simple_leg::simple_leg(){
    // joint state interfaceとの連携
    hardware_interface::JointStateHandle hip_state_handle("Hip", &pos[0], &vel[0], &eff[0]);
    joint_state_interface.registerHandle(hip_state_handle);
    hardware_interface::JointStateHandle knee_state_handle("Knee", &pos[1], &vel[1], &eff[1]);
    joint_state_interface.registerHandle(knee_state_handle);
    registerInterface(&joint_state_interface);

    // 脚先位置コントローラとの連携
    hardware_interface::LegPositionHandle leg_position_handle("FL", &x, &z);
    leg_position_interface.registerHandle(leg_position_handle);
    registerInterface(&leg_position_interface);
}

// 指定された脚先位置を実現するための逆運動学を計算する
void simple_leg::solve_IK(void){
    double L = std::hypot(x, z);

    pos_cmd[0] = -atan2(x, -z) + acos((pow(L, 2) + pow(l1, 2) - pow(l2, 2)) / (2 * L * l1));
    pos_cmd[1] = -M_PI + acos((pow(l1, 2.0) + pow(l2, 2.0) - pow(L, 2)) / (2 * l1 * l2));
}
 
// 実機ロボットから応答値を受け取る
void simple_leg::read(ros::Time time, ros::Duration period){
    // send_command()
    // 今回は逆運動学計算で求めた結果をjoint_stateで返す
    pos[0] = pos_cmd[0];
    pos[1] = pos_cmd[1];
}


// 実機ロボットに指令値を送る
void simple_leg::write(ros::Time time, ros::Duration period){
    // receive_reply()
    solve_IK();
}

}