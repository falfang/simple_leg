#ifndef SIMPLE_LEG_HW_SIM_H_
#define SIMPLE_LEG_HW_SIM_H_

#include <vector>
#include <string>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <simple_leg_controllers/leg_position_interface.h>
#include <pluginlib/class_list_macros.hpp>
#include <angles/angles.h>

// Gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

// gazebo_ros_control
#include <gazebo_ros_control/robot_hw_sim.h>

// URDF
#include <urdf/model.h>
#include <simple_leg_controllers/simple_leg_spec.h>

namespace simple_leg_hw_sim{

class SimpleLegHWSim : public gazebo_ros_control::RobotHWSim{
private:
    unsigned int n_dof_;

    std::vector<std::string> joint_names_;
    std::vector<double> joint_position_;
    std::vector<double> joint_velocity_;
    std::vector<double> joint_effort_;
    std::vector<double> joint_position_command_;
    std::vector<int> joint_types_;
    double cmd_x, cmd_z;

    std::vector<gazebo::physics::JointPtr> sim_joints_;
    std::string physics_type_;

    // Hardware interface: joints
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::LegPositionInterface leg_position_interface_;

    // Joint limits interface
    joint_limits_interface::PositionJointSoftLimitsInterface jnt_limits_interface_;

    void solve_IK(void);

public:
    SimpleLegHWSim();

    bool initSim(const std::string&, ros::NodeHandle, gazebo::physics::ModelPtr, const urdf::Model* const, std::vector<transmission_interface::TransmissionInfo>) override;
    void readSim(ros::Time, ros::Duration) override;
    void writeSim(ros::Time, ros::Duration) override;
};

typedef boost::shared_ptr<SimpleLegHWSim> SimpleLegHWSimPtr;

}   // namespace simple_leg_sim_hw

#endif  // SIMPLE_LEG_HW_SIM_H_