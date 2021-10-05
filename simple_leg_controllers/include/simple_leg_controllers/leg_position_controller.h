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
#include <simple_leg_controllers/leg_position_interface.h>
#include <simple_leg_msgs/leg_command.h>
#include <pluginlib/class_list_macros.hpp>
#include <realtime_tools/realtime_publisher.h>

namespace simple_leg_controllers{

class LegPositionController : public controller_interface::Controller<hardware_interface::LegPositionInterface>{
private:
    hardware_interface::LegPositionHandle leg_;
    ros::Subscriber sub_command;

    struct Command{
        double x;
        double z;
    };

    Command cmd;
    void commandCB(const simple_leg_msgs::leg_commandConstPtr&);

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