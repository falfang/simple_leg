/**
 * @file leg_position_controller.h
 * @author your name (you@domain.com)
 * @brief 自作ros_controller
 * @version 0.1
 * @date 2021-10-05
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include <controller_interface/controller.h>
#include <ros_control_tutorial/leg_position_interface.h>
#include <pluginlib/class_list_macros.hpp>
#include <ros_control_tutorial/leg_command.h>
#include <realtime_tools/realtime_publisher.h>

namespace leg_position_controller{

class LegPositionController : public controller_interface::Controller<hardware_interface::LegPositionInterface>{
private:
    hardware_interface::LegPositionHandle leg_;
    ros::Subscriber sub_command;

    struct Command{
        double x;
        double z;
    };

    Command cmd;
    void setCommandCB(const ros_control_tutorial::leg_commandConstPtr&);

public:
    LegPositionController(void);
    ~LegPositionController();

    virtual bool init(hardware_interface::LegPositionInterface* hw, ros::NodeHandle& nh);
    virtual void starting(const ros::Time&);
    virtual void update(const ros::Time&, const ros::Duration&);
    virtual void stopping(const ros::Time&);

    std::string getLegName(void);
    void enforceWorkspaceLimit(Command&){};
};



}