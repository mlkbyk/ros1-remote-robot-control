#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <my_robot_msgs/ExecuteMotionAction.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <string>
#include <cmath>

class MotionServer {
public:
  MotionServer()
  : as_(nh_, "/execute_motion",
        boost::bind(&MotionServer::executeCB, this, _1),
        false) {

    nh_.param<std::string>("cmd_topic", cmd_topic_, "/cmd_vel");
    nh_.param<std::string>("odom_topic", odom_topic_, "/odom");

    cmd_pub_  = nh_.advertise<geometry_msgs::Twist>(cmd_topic_, 10);
    odom_sub_ = nh_.subscribe(odom_topic_, 10, &MotionServer::odomCB, this);

    as_.start();

    ROS_INFO("MotionServer ready: /execute_motion");
    ROS_INFO("MotionServer topics: cmd=%s odom=%s", cmd_topic_.c_str(), odom_topic_.c_str());
  }

private:
  void odomCB(const nav_msgs::Odometry::ConstPtr& msg) {
    last_odom_ = *msg;
    got_odom_ = true;
  }

  void stopRobot() {
    geometry_msgs::Twist z;
    cmd_pub_.publish(z);
  }

  static double normalizeAngle(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  double getYawFromOdom() const {
    const auto& q = last_odom_.pose.pose.orientation;
    // yaw from quaternion (planar assumption)
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  void executeCB(const my_robot_msgs::ExecuteMotionGoalConstPtr& goal) {
    my_robot_msgs::ExecuteMotionFeedback fb;
    my_robot_msgs::ExecuteMotionResult res;

    const uint8_t MODE_STOP  = 0;
    const uint8_t MODE_TURN  = 1;   // v1 = degrees
    const uint8_t MODE_DRIVE = 2;   // v1 = meters
    const uint8_t MODE_VEL   = 3;   // v1 = linear, v2 = angular

    ROS_INFO("Goal: mode=%u v1=%.2f v2=%.2f", goal->mode, goal->v1, goal->v2);

    if (as_.isPreemptRequested() || !ros::ok()) {
      stopRobot();
      res.ok = false; res.message = "Preempted";
      as_.setPreempted(res);
      return;
    }

    // STOP
    if (goal->mode == MODE_STOP) {
      stopRobot();
      res.ok = true; res.message = "Stopped";
      as_.setSucceeded(res);
      return;
    }

    // VEL (2 second burst)
    if (goal->mode == MODE_VEL) {
      geometry_msgs::Twist cmd;
      cmd.linear.x  = goal->v1;
      cmd.angular.z = goal->v2;

      ros::Rate r(20);
      for (int i = 0; i < 40 && ros::ok(); ++i) { // ~2 sec
        if (as_.isPreemptRequested()) {
          stopRobot();
          res.ok = false; res.message = "Canceled";
          as_.setPreempted(res);
          return;
        }
        cmd_pub_.publish(cmd);
        fb.progress = static_cast<float>((i + 1) / 40.0);
        fb.state = "vel";
        as_.publishFeedback(fb);
        r.sleep();
      }
      stopRobot();
      res.ok = true; res.message = "VEL done";
      as_.setSucceeded(res);
      return;
    }

    // TURN/DRIVE need odom
    if (!got_odom_) {
      stopRobot();
      res.ok = false;
      res.message = "No odom yet. TURN/DRIVE require odom.";
      as_.setAborted(res);
      return;
    }

    // TURN: v1 = degrees (positive left, negative right)
    if (goal->mode == MODE_TURN) {
      const double target_deg = goal->v1;
      const double target_rad = target_deg * M_PI / 180.0;

      const double yaw0 = getYawFromOdom();
      const double yaw_target = normalizeAngle(yaw0 + target_rad);

      // Slow, safe angular speed
      const double w = (target_rad >= 0.0) ? 0.25 : -0.25; // rad/s

      geometry_msgs::Twist cmd;
      cmd.angular.z = w;

      ros::Rate r(30);
      const double timeout_s = 12.0;
      const ros::Time t0 = ros::Time::now();

      while (ros::ok()) {
        if (as_.isPreemptRequested()) {
          stopRobot();
          res.ok = false; res.message = "Canceled";
          as_.setPreempted(res);
          return;
        }

        const double yaw = getYawFromOdom();
        const double err = normalizeAngle(yaw_target - yaw);

        fb.state = "turn";
        double denom = std::max(1e-6, std::fabs(target_rad));
        fb.progress = static_cast<float>(1.0 - std::min(1.0, std::fabs(err) / denom));
        as_.publishFeedback(fb);

        // Stop when close enough (~3 degrees)
        if (std::fabs(err) < 0.05) break;

        if ((ros::Time::now() - t0).toSec() > timeout_s) {
          stopRobot();
          res.ok = false; res.message = "TURN timeout";
          as_.setAborted(res);
          return;
        }

        cmd_pub_.publish(cmd);
        r.sleep();
      }

      stopRobot();
      res.ok = true; res.message = "TURN done";
      as_.setSucceeded(res);
      return;
    }

    // DRIVE: v1 = meters (positive forward, negative backward)
    if (goal->mode == MODE_DRIVE) {
      const double dist_m = goal->v1;
      const double speed  = (dist_m >= 0.0) ? 0.06 : -0.06; // m/s slow safe

      const double x0 = last_odom_.pose.pose.position.x;
      const double y0 = last_odom_.pose.pose.position.y;

      geometry_msgs::Twist cmd;
      cmd.linear.x = speed;

      ros::Rate r(30);
      const double timeout_s = 20.0;
      const ros::Time t0 = ros::Time::now();

      while (ros::ok()) {
        if (as_.isPreemptRequested()) {
          stopRobot();
          res.ok = false; res.message = "Canceled";
          as_.setPreempted(res);
          return;
        }

        const double x = last_odom_.pose.pose.position.x;
        const double y = last_odom_.pose.pose.position.y;
        const double traveled = std::sqrt((x - x0)*(x - x0) + (y - y0)*(y - y0));

        fb.state = "drive";
        fb.progress = static_cast<float>(std::min(1.0, traveled / std::max(1e-6, std::fabs(dist_m))));
        as_.publishFeedback(fb);

        if (traveled >= std::fabs(dist_m)) break;

        if ((ros::Time::now() - t0).toSec() > timeout_s) {
          stopRobot();
          res.ok = false; res.message = "DRIVE timeout";
          as_.setAborted(res);
          return;
        }

        cmd_pub_.publish(cmd);
        r.sleep();
      }

      stopRobot();
      res.ok = true; res.message = "DRIVE done";
      as_.setSucceeded(res);
      return;
    }

    // Unknown mode
    stopRobot();
    res.ok = false;
    res.message = "Unknown mode.";
    as_.setAborted(res);
  }

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<my_robot_msgs::ExecuteMotionAction> as_;
  ros::Publisher cmd_pub_;
  ros::Subscriber odom_sub_;

  std::string cmd_topic_;
  std::string odom_topic_;

  nav_msgs::Odometry last_odom_;
  bool got_odom_ = false;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "motion_server");
  MotionServer s;
  ros::spin();
  return 0;
}
