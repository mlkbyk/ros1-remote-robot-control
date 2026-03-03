#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <my_robot_msgs/ExecuteMotionAction.h>

#include <string>
#include <cstdlib>

using Client = actionlib::SimpleActionClient<my_robot_msgs::ExecuteMotionAction>;

static void PrintUsage() {
  ROS_INFO("Usage:");
  ROS_INFO("  rosrun motion_client_pkg motion_cli stop");
  ROS_INFO("  rosrun motion_client_pkg motion_cli vel  --lin 0.10 --ang 0.30");
  ROS_INFO("  rosrun motion_client_pkg motion_cli turn --deg 90");
  ROS_INFO("  rosrun motion_client_pkg motion_cli drive --m 1.0");
  ROS_INFO("");
  ROS_INFO("Mode mapping:");
  ROS_INFO("  stop  -> mode=0");
  ROS_INFO("  turn  -> mode=1 (v1=degrees)");
  ROS_INFO("  drive -> mode=2 (v1=meters)");
  ROS_INFO("  vel   -> mode=3 (v1=linear, v2=angular)");
}

static bool GetFlagValue(int argc, char** argv, const std::string& flag, double& out) {
  for (int i = 2; i + 1 < argc; ++i) {
    if (std::string(argv[i]) == flag) {
      out = std::atof(argv[i + 1]);
      return true;
    }
  }
  return false;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "motion_cli");

  if (argc < 2) {
    PrintUsage();
    return 1;
  }

  const std::string command = argv[1];

  uint8_t mode = 0;
  double v1 = 0.0;
  double v2 = 0.0;

  if (command == "stop") {
    mode = 0;
  } else if (command == "vel") {
    mode = 3;
    GetFlagValue(argc, argv, "--lin", v1);
    GetFlagValue(argc, argv, "--ang", v2);
  } else if (command == "turn") {
    mode = 1;
    if (!GetFlagValue(argc, argv, "--deg", v1)) {
      ROS_ERROR("Missing required flag: --deg");
      PrintUsage();
      return 2;
    }
  } else if (command == "drive") {
    mode = 2;
    if (!GetFlagValue(argc, argv, "--m", v1)) {
      ROS_ERROR("Missing required flag: --m");
      PrintUsage();
      return 2;
    }
  } else {
    ROS_ERROR("Unknown command: %s", command.c_str());
    PrintUsage();
    return 2;
  }

  Client ac("/execute_motion", true);
  ROS_INFO("Waiting for action server: /execute_motion ...");
  ac.waitForServer();
  ROS_INFO("Connected.");

  my_robot_msgs::ExecuteMotionGoal goal;
  goal.mode = mode;
  goal.v1 = static_cast<float>(v1);
  goal.v2 = static_cast<float>(v2);

  ROS_INFO("Sending goal: cmd=%s mode=%u v1=%.2f v2=%.2f",
           command.c_str(), mode, v1, v2);

  ac.sendGoal(goal);
  const bool finished = ac.waitForResult(ros::Duration(10.0));

  if (!finished) {
    ROS_WARN("Timeout waiting for result. Cancelling goal...");
    ac.cancelGoal();
    return 3;
  }

  const auto state = ac.getState();
  const auto res = ac.getResult();

  ROS_INFO("Result state: %s", state.toString().c_str());
  if (res) {
    ROS_INFO("ok=%s message=%s", res->ok ? "true" : "false", res->message.c_str());
  } else {
    ROS_WARN("No result received.");
  }

  return 0;
}


