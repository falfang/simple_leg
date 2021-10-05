/**
 * @file simple_leg_hw.h
 * @author your name (you@domain.com)
 * @brief 自作ros_controllerをテストするための実機用RobotHW
 * @version 0.1
 * @date 2021-10-05
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <ros/ros.h>
#include <ros_control_tutorial/leg_position_interface.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <ros_control_tutorial/simple_leg_spec.h>

namespace simple_leg{

class simple_leg : public hardware_interface::RobotHW{
private:
    hardware_interface::LegPositionInterface leg_position_interface;
    hardware_interface::JointStateInterface joint_state_interface;
    double x, z;
    double pos_cmd[2];
    double pos[2], vel[2], eff[2];
    void solve_IK(void);

public:
    simple_leg();
    ros::NodeHandle nh;

    ros::Time getTime() const { return ros::Time::now(); }
    ros::Duration getPeriod() const { return ros::Duration(0.01); }

    void read(ros::Time, ros::Duration);
    void write(ros::Time, ros::Duration);

};

}