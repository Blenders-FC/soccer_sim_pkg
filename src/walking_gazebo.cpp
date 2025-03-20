// op3_manager converted from ini pose to walking

/* ROS API Header */
#include <ros/ros.h>
#include <std_msgs/String.h>

/* ROBOTIS Controller Header */
#include "robotis_controller/robotis_controller.h"

/* Sensor Module Header */
#include "open_cr_module/open_cr_module.h"

/* Motion Module Header */
#include "op3_base_module/base_module.h"
#include "op3_head_control_module/head_control_module.h"
#include "op3_action_module/action_module.h"
#include "op3_walking_module/op3_walking_module.h"
#include "op3_direct_control_module/direct_control_module.h"
#include "op3_online_walking_module/online_walking_module.h"
#include "op3_tuning_module/tuning_module.h"

using namespace robotis_framework;
using namespace robotis_op;

int g_baudrate;
const int BAUD_RATE = 2000000;
const std::string SUB_CONTROLLER_DEVICE = "/dev/ttyUSB0";

ros::Publisher g_init_pose_pub;
ros::Publisher g_demo_command_pub;
ros::Publisher g_walking_pub;
ros::Publisher g_action_pub;

std::string g_offset_file;
std::string g_robot_file;
std::string g_init_file;
std::string g_device_name;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "op3_walking");
  ros::NodeHandle nh;

  ROS_INFO("op3_walking->init");
  RobotisController *controller = RobotisController::getInstance();

  nh.param<std::string>("offset_file_path", g_offset_file, "");
  nh.param<std::string>("robot_file_path", g_robot_file, "");
  nh.param<std::string>("init_file_path", g_init_file, "");
  nh.param<std::string>("device_name", g_device_name, SUB_CONTROLLER_DEVICE);
  nh.param<int>("baud_rate", g_baudrate, BAUD_RATE);

  g_init_pose_pub = nh.advertise<std_msgs::String>("/robotis/base/ini_pose", 0);
  g_demo_command_pub = nh.advertise<std_msgs::String>("/ball_tracker/command", 0);
  g_walking_pub = nh.advertise<std_msgs::String>("/robotis/walking/command", 0);
  g_action_pub = nh.advertise<std_msgs::Int32>("/robotis/action/page_num", 0);

  nh.param<bool>("gazebo", controller->gazebo_mode_, false);

  ROS_WARN("SET TO GAZEBO MODE!");
  std::string robot_name;
  nh.param<std::string>("gazebo_robot_name", robot_name, "");
  if (robot_name != "") controller->gazebo_robot_name_ = robot_name;
    
  if (g_robot_file == "")
  {
    ROS_ERROR("NO robot file path in the ROS parameters.");
    return -1;
  }

  // initialize robot
  if (controller->initialize(g_robot_file, g_init_file) == false)
  {
    ROS_ERROR("ROBOTIS Controller Initialize Fail!");
    return -1;
  }

  // load offset
  if (g_offset_file != "")
    controller->loadOffset(g_offset_file);

  usleep(300 * 1000);

  /* Add Sensor Module */
  controller->addSensorModule((SensorModule*) OpenCRModule::getInstance());
  ROS_INFO("op3_walking->init");

  /* Add Motion Module */
  controller->addMotionModule((MotionModule*) ActionModule::getInstance());
  controller->addMotionModule((MotionModule*) BaseModule::getInstance());
  controller->addMotionModule((MotionModule*) HeadControlModule::getInstance());
  controller->addMotionModule((MotionModule*) WalkingModule::getInstance());
  controller->addMotionModule((MotionModule*) OnlineWalkingModule::getInstance());

  // start timer
  controller->startTimer();
  usleep(100 * 1000);

  // go to init pose
  std_msgs::String init_msg;
  init_msg.data = "ini_pose";

  g_init_pose_pub.publish(init_msg);
  ROS_INFO("Go to init pose");
  usleep(100 * 1000);

  // Set the control module to walking

  controller->setCtrlModule("walking_module");
  usleep(1000 * 1000);

  std_msgs::String walking_msg;
  walking_msg.data = "start";

  g_walking_pub.publish(walking_msg);
  ROS_INFO("Start walking");

  usleep(100 * 1000);

  while (ros::ok())
  {
    usleep(100 * 1000);
    g_walking_pub.publish(walking_msg);
    ros::spinOnce();
  }

  return 0;
}