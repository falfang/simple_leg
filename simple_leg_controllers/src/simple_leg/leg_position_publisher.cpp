#include <ros/ros.h>
#include <simple_leg_msgs/leg_command.h>

simple_leg_msgs::leg_command cmd;

int main(int argc, char** argv){
    ros::init(argc, argv, "leg_position_publisher");
    ros::NodeHandle nh;
    ros::Rate loop_rate(100);
    ros::Publisher pub = nh.advertise<simple_leg_msgs::leg_command>("/leg_position_controller/command", 1);

    cmd.x = 0.01;
    cmd.z = -0.3;

    while(ros::ok()){
        pub.publish(cmd);
        loop_rate.sleep();
    }
    return 0;
}