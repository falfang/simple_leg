/**
 * @file simple_leg_control.cpp
 * @author your name (you@domain.com)
 * @brief 実機用のRobotHW本体
 * @version 0.1
 * @date 2021-10-05
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <ros_control_tutorial/simple_leg_hw.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "simple_leg");

    simple_leg::simple_leg leg;
    controller_manager::ControllerManager cm(&leg, leg.nh);

    ros::Rate rate(1/leg.getPeriod().toSec());
    ros::AsyncSpinner spinner(1);
    spinner.start();

    while(ros::ok()){
        ros::Time now = leg.getTime();
        ros::Duration dt = leg.getPeriod();

        leg.read(now, dt);
        cm.update(now, dt);
        leg.write(now, dt);
        rate.sleep();
    }
    spinner.stop();
    return 0;
}