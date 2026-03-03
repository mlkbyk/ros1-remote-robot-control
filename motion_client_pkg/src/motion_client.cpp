#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <my_robot_msgs/ExecuteMotionAction.h>

using Client = actionlib::SimpleActionClient<my_robot_msgs::ExecuteMotionAction>;

static void feedbackCb(const my_robot_msgs::ExecuteMotionFeedbackConstPtr& fb){
  ROS_INFO("Feedback: progress=%.2f state=%s", fb->progress, fb->state.c_str());
}

int main(int argc, char** argv){
  ros::init(argc, argv, "motion_client");
  ros::NodeHandle nh;

  // args: mode v1 v2
  if (argc < 2) {
    ROS_ERROR("Usage: rosrun motion_client_pkg motion_client <mode> [v1] [v2]");
    ROS_ERROR("Modes: 0=STOP, 3=VEL (v1=linear, v2=angular)");
    return 1;
  }

  int mode = std::atoi(argv[1]);
  float v1 = (argc >= 3) ? std::atof(argv[2]) : 0.0f;
  float v2 = (argc >= 4) ? std::atof(argv[3]) : 0.0f;

  Client ac("/execute_motion", true);
  ROS_INFO("Waiting for /execute_motion action server...");
  ac.waitForServer();
  ROS_INFO("Connected.");

  my_robot_msgs::ExecuteMotionGoal goal;
  goal.mode = (uint8_t)mode;
  goal.v1 = v1;
  goal.v2 = v2;

  ROS_INFO("Sending goal: mode=%d v1=%.2f v2=%.2f", mode, v1, v2);

  ac.sendGoal(goal,
              Client::SimpleDoneCallback(),
              Client::SimpleActiveCallback(),
              feedbackCb);

  bool finished = ac.waitForResult(ros::Duration(10.0));
  if (!finished){
    ROS_WARN("Timeout, canceling goal...");
    ac.cancelGoal();
    return 2;
  }

  auto state = ac.getState();
  auto res = ac.getResult();
  ROS_INFO("Result state: %s", state.toString().c_str());
  if (res){
    ROS_INFO("ok=%s message=%s", res->ok ? "true" : "false", res->message.c_str());
  }
  return 0;
}
