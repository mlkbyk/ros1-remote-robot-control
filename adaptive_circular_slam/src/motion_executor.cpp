#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

nav_msgs::Path current_path;
bool path_received = false;

MoveBaseClient* ac_ptr;

// PATH CALLBACK
void pathCallback(const nav_msgs::Path::ConstPtr& msg)
{
    current_path = *msg;
    path_received = true;
}

// SCORE CALLBACK
void scoreCallback(const std_msgs::Float64::ConstPtr& msg)
{
    if(!path_received) return;

    ROS_INFO("Received score: %.2f", msg->data);

    // Path içinden ortadaki waypointi seçiyoruz
    int mid_index = current_path.poses.size() / 2;

    geometry_msgs::PoseStamped target_pose =
        current_path.poses[mid_index];

    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose = target_pose.pose;

    ROS_INFO("Sending goal to move_base...");

    ac_ptr->sendGoal(goal);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "motion_executor_node");
    ros::NodeHandle nh;

    MoveBaseClient ac("move_base", true);
    ac_ptr = &ac;

    ROS_INFO("Waiting for move_base server...");
    ac.waitForServer();

    ros::Subscriber path_sub =
        nh.subscribe("/circle_path", 10, pathCallback);

    ros::Subscriber score_sub =
        nh.subscribe("/circle_score", 10, scoreCallback);

    ROS_INFO("Motion executor started.");

    ros::spin();
    return 0;
}
