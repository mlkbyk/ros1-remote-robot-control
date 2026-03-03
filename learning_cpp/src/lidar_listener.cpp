#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

// Lidar verisi geldikce bu fonksiyon calisacak
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    // 1. Veri paketinde toplam kac nokta var? (360? 720?)
    int n = msg->ranges.size();

    // 2. Ipuclarini topla: Basi (0) ve Ortasi (n/2)
    // Robotun onune bir sey koydugunda hangisi kuculuyor bakacagiz.
    float aday_bas = msg->ranges[0];
    float aday_orta = msg->ranges[n / 2];

    // 3. Ekrana yazdir
    ROS_INFO("Toplam: %d | Index 0: %.2f m | Index Orta: %.2f m", n, aday_bas, aday_orta);
}

int main(int argc, char **argv)
{
    // ROS Baslat
    ros::init(argc, argv, "lidar_dedektifi");
    ros::NodeHandle nh;

    // Dinlemeye Basla (/scan konusunu dinle)
    ros::Subscriber sub = nh.subscribe("/scan", 1000, scanCallback);

    // Kapanma, surekli dinle
    ros::spin();

    return 0;
}
