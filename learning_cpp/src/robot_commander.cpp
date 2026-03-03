#include "geometry_msgs/Twist.h"
#include "ros/ros.h"

int main(int argc, char **argv ){
    ros::init(argc,argv,"commander_node");
    ros::NodeHandle nh;
    ros::Publisher pub=nh.advertise<geometry_msgs::Twist>("/cmd_vel",1000);
    ros::Duration(1.0).sleep(); // wait for publisher to set up

    geometry_msgs::Twist velocity;
    const double wheelbase=0.2; // distance between wheels in meters
    velocity.linear.x=0.15; // forward speed in m/s
    double aim_radius=0.6; // desired turning radius in meters
    velocity.angular.z=velocity.linear.x/aim_radius; // angular speed in rad/s

    ros::Rate loop_rate(10);
    ROS_INFO("Robot Starting Motion! (Speed: 0.15 m/s, Radius: 0.6 m)");
    ROS_INFO("Press 'Ctrl+C' in the terminal to stop.");

    while(ros::ok()){
        pub.publish(velocity);
        ros::spinOnce();
        loop_rate.sleep();

        
        
    }
    // --- SAFETY SECTION ---
    // This part runs automatically ONLY when Ctrl+C is pressed.
    ROS_INFO("Stopping command received. Halting robot...");
    
    //set velocity to zero
    velocity.linear.x=0.0;
    velocity.angular.z=0.0;

    pub.publish(velocity);
    ros::Duration(0.1).sleep();


    return 0;


}

