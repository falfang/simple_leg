#include <ros/ros.h>
#include <simple_leg_msgs/leg_command.h>

simple_leg_msgs::leg_command cmd;

int main(int argc, char** argv){
    ros::init(argc, argv, "leg_position_publisher");
    ros::NodeHandle nh;
    ros::Rate loop_rate(100);
    ros::Publisher pub = nh.advertise<simple_leg_msgs::leg_command>("/leg_position_controller/command", 1);

    double x0 = 0.0;
    double z0 = -0.25;

    // parameters for circle trajectory
    double freq = 1.0;
    double radius = 0.1;
    
    cmd.x = x0;
    cmd.z = z0;

    ros::Time t_start = ros::Time::now();

    while(ros::ok()){
        double current_time = (ros::Time::now() - t_start).toSec();

        // move foot along circle trajectory
        cmd.x = x0 + radius*cos(2*M_PI*freq*current_time - (3*M_PI/2));
        cmd.z = z0 + radius*sin(2*M_PI*freq*current_time - (3*M_PI/2));
        
        pub.publish(cmd);
        loop_rate.sleep();
    }
    return 0;
}