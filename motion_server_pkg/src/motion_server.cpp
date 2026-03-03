#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <my_robot_msgs/ExecuteMotionAction.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <string>

class MotionServer {
public:
  MotionServer()
  : as_(nh_, "/execute_motion",
        boost::bind(&MotionServer::executeCB, this, _1),
        false) {

    nh_.param<std::string>("cmd_topic", cmd_topic_, "/cmd_vel");
    nh_.param<std::string>("odom_topic", odom_topic_, "/odom");

    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_topic_, 10);
    odom_sub_ = nh_.subscribe(odom_topic_, 10, &MotionServer::odomCB, this);

    as_.start();

    ROS_INFO("MotionServer ready: /execute_motion");
    ROS_INFO("MotionServer topics: cmd=%s odom=%s", cmd_topic_.c_str(), odom_topic_.c_str());
  }

private:
  void odomCB(const nav_msgs::Odometry::ConstPtr&) { got_odom_ = true; }

  void stopRobot() {
    geometry_msgs::Twist z;
    cmd_pub_.publish(z);
  }

  void executeCB(const my_robot_msgs::ExecuteMotionGoalConstPtr& goal) {
    my_robot_msgs::ExecuteMotionFeedback fb;
    my_robot_msgs::ExecuteMotionResult res;

    const uint8_t MODE_STOP  = 0;
    const uint8_t MODE_TURN  = 1;
    const uint8_t MODE_DRIVE = 2;
    const uint8_t MODE_VEL   = 3;

    ROS_INFO("Goal: mode=%u v1=%.2f v2=%.2f", goal->mode, goal->v1, goal->v2);

    if (as_.isPreemptRequested() || !ros::ok()) {
      stopRobot();
      res.ok = false; res.message = "Preempted";
      as_.setPreempted(res);
      return;
    }

    if (goal->mode == MODE_STOP) {
      stopRobot();
      res.ok = true; res.message = "Stopped";
      as_.setSucceeded(res);
      return;
    }

    if (goal->mode == MODE_VEL) {
      geometry_msgs::Twist cmd;
      cmd.linear.x  = goal->v1;
      cmd.angular.z = goal->v2;

      ros::Rate r(20);
      for (int i = 0; i < 40 && ros::ok(); ++i) { // ~2 sec burst
        if (as_.isPreemptRequested()) {
          stopRobot();
          res.ok = false; res.message = "Canceled";
          as_.setPreempted(res);
          return;
        }
        cmd_pub_.publish(cmd);
        fb.progress = (i + 1) / 40.0f;
        fb.state = "vel";
        as_.publishFeedback(fb);
        r.sleep();
      }
      stopRobot();
      res.ok = true; res.message = "VEL done";
      as_.setSucceeded(res);
      return;
    }

    if (!got_odom_) {
      stopRobot();
      res.ok = false;
      res.message = "No odom yet. TURN/DRIVE require odom.";
      as_.setAborted(res);
      return;
    }

    stopRobot();
    res.ok = false;
    res.message = "TURN/DRIVE not implemented yet.";
    as_.setAborted(res);
  }

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<my_robot_msgs::ExecuteMotionAction> as_;
  ros::Publisher cmd_pub_;
  ros::Subscriber odom_sub_;

  std::string cmd_topic_;
  std::string odom_topic_;

  bool got_odom_ = false;
};

int main(int argc, char** argv){
  ros::init(argc, argv, "motion_server");
  MotionServer s;
  ros::spin();
  return 0;
}
