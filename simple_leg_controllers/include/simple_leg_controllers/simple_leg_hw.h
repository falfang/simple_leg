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
#include <simple_leg_controllers/leg_position_interface.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <simple_leg_controllers/simple_leg_spec.h>
#include <std_msgs/Float64.h>

namespace simple_leg{

class simple_leg : public hardware_interface::RobotHW{
private:
    const unsigned int n_dof_ = 2;

    hardware_interface::LegPositionInterface leg_position_interface;
    hardware_interface::JointStateInterface joint_state_interface;
    double cmd_x, cmd_z;

    std_msgs::Float64 hip_cmd, knee_cmd;
    ros::Publisher hip_cmd_pub, knee_cmd_pub;

    std::vector<double> jnt_pos_;
    std::vector<double> jnt_vel_;
    std::vector<double> jnt_eff_;

    std::vector<double> jnt_pos_cmd_;

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