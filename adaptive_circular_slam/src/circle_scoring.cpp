#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Float64.h>

nav_msgs::OccupancyGrid current_map;
bool map_received = false;

ros::Publisher score_pub;

// MAP CALLBACK
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    current_map = *msg;
    map_received = true;
}

// PATH CALLBACK
void pathCallback(const nav_msgs::Path::ConstPtr& msg)
{
    if(!map_received) return;

    double score = 0.0;

    for(auto pose : msg->poses)
    {
        int mx = (pose.pose.position.x - current_map.info.origin.position.x) 
                 / current_map.info.resolution;

        int my = (pose.pose.position.y - current_map.info.origin.position.y) 
                 / current_map.info.resolution;

        int index = my * current_map.info.width + mx;

        if(index >= 0 && index < current_map.data.size())
        {
            if(current_map.data[index] == 0)
                score += 1.0; // boş alan
            else
                score -= 5.0; // obstacle ceza
        }
    }

    std_msgs::Float64 result;
    result.data = score;

    score_pub.publish(result);

    ROS_INFO("Circle Score: %.2f", score);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "circle_scoring_node");
    ros::NodeHandle nh;

    ros::Subscriber map_sub =
        nh.subscribe("/map", 10, mapCallback);

    ros::Subscriber path_sub =
        nh.subscribe("/circle_path", 10, pathCallback);

    score_pub =
        nh.advertise<std_msgs::Float64>("circle_score", 10);

    ros::spin();
    return 0;
}
