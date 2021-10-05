#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <map>
#include <string>
#include <vector>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32MultiArray.h>

class TRobo : public hardware_interface::RobotHW{
public:
    TRobo();
    ros::NodeHandle nh;

    ros::Time getTime() const { return ros::Time::now(); }
    ros::Duration getPeriod() const { return ros::Duration(0.01) ;} // 0.01sで回す
    void timeCallback(const std_msgs::Float32::ConstPtr& msg_sub);

    ros::Publisher pub;
    ros::Subscriber sub;

    void read(ros::Time, ros::Duration);    // 実機のロボットデータをros_controlに渡す関数
    void write(ros::Time, ros::Duration);   // ros_controlからの指令を実機のロボットに渡す関数

protected:
    // ros_controlで使いたいhardware_interfaceを宣言しておく必要がある
    hardware_interface::JointStateInterface joint_state_interface;
    hardware_interface::PositionJointInterface joint_position_interface;

    double cmd_[1];
    double pos_[1];
    double vel_[1];
    double eff_[1];
};