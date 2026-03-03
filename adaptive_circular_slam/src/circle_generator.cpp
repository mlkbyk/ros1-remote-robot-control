#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <math.h>

ros::Publisher path_pub;

nav_msgs::Path generateCircle(double radius)
{
    nav_msgs::Path path;
    path.header.frame_id = "map";

    int points = 36;

    for(int i=0; i<points; i++)
    {
        double angle = 2*M_PI * i / points;

        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = radius * cos(angle);
        pose.pose.position.y = radius * sin(angle);
        pose.pose.orientation.w = 1.0;

        path.poses.push_back(pose);
    }

    return path;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "circle_generator_node");
    ros::NodeHandle nh;

    path_pub = nh.advertise<nav_msgs::Path>("circle_path", 10);

    ros::Rate rate(1);

    while(ros::ok())
    {
        nav_msgs::Path path = generateCircle(1.0); // şimdilik 1m
        path_pub.publish(path);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
