#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    double x_position=msg->pose.pose.position.x;
    double y_position=msg->pose.pose.position.y;

    double linear_vel=msg->twist.twist.linear.x;
    double angular_vel=msg->twist.twist.angular.z;

    ROS_INFO("---------------------------------------------");
    ROS_INFO("Position -> X: [%.2f] m | Y: [%.2f] m",x_position, y_position);
    ROS_INFO("Velocity-> LINEAR: [%.2f] m/s | Angular: [%.2f] rad/s", linear_vel, angular_vel);
    

}

int main(int argc, char **argv){
    
    ros::init(argc,argv,"odom_listener_node");
    ros::NodeHandle nh;
    ros::Subscriber sub=nh.subscribe("/odom",1000,odomCallback);

    ROS_INFO("Odom Listener Node Started! Waiting for robot data...");
    ros::spin();
    return 0;

}