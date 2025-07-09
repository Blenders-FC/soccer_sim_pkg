// op3_manager converted from ini pose to walking

/* ROS API Header */
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/JointState.h>

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
ros::Subscriber red_loc;

std::string g_offset_file;
std::string g_robot_file;
std::string g_init_file;
std::string g_device_name;

double head_pan_;
double head_tilt_;
double angle_mov_x_;
double angle_mov_y_;
double xerror_;
double yerror_;
double box_x;
double box_y;
sensor_msgs::JointState write_msg_;
const double CAMERA_HEIGHT_ = 0.46;
const double hip_pitch_offset_ = 0.12217305; //7°
double head_pan_angle_;
double head_tilt_angle_;

void locCallback(const geometry_msgs::Point& msg){
  box_x = msg.x;
  box_y = msg.y;
}

double getBoxX(void){
  return box_x;
}

double getBoxY(void)
{
  return box_y;
}
double getHeadPan(){
  return head_pan_;
}

double getHeadTilt(){
  return head_tilt_;
}

void jointStatesCallback( const sensor_msgs::JointState& msg){
  head_pan_ = msg.position[0];
  head_tilt_ = msg.position[1];
}


void Center(){
    head_pan_angle_ = getHeadPan();
    head_tilt_angle_ = getHeadTilt();
    locCallback();
    double box_x_= getBoxX();
    double box_y_= getBoxY();
    angle_mov_x_ = head_pan_angle_ * 57.2958;   // RadToDeg -> 180/pi
    angle_mov_y_ = head_tilt_angle_ * 57.2958;  // RadToDeg -> 180/pi
    xerror_ = (320 - box_x_) * 0.21875;  // 70 / 320
    yerror_ = (240 - box_y_) * 0.29166;  // 70 / 240

    if (xerror_ >= 11 || xerror_ <= -11)
    {
        xerror_ = xerror_ * M_PI / 180;

        if (xerror_ < 0)
        {
          angle_mov_x_ -= 1;
        }
        else
        {
          angle_mov_x_ += 1;
        }
        writeHeadJoint(angle_mov_x_, true);
        //walkTowardsBall(current_ball_pan, head_tilt);
    }
    else if (yerror_ >= 20 || yerror_ <= -20)
    {
        yerror_ = yerror_ * M_PI / 180;

        if (yerror_ < 0)
        {
          angle_mov_y_ -= 1;
        }
        else
        {
          angle_mov_y_ += 1;
        }
        writeHeadJoint(angle_mov_y_, false);
      }}


void writeHeadJoint(double ang_value, bool is_pan){
  if (getModule("r_knee") != "direct_control_module")
  {
      setModule("direct_control_module");
      ros::Duration(1).sleep();
  }
  write_msg_;
  write_msg_.header.stamp = ros::Time::now();
      
  ang_value *= 0.0174533;  // DegToRad -> pi/180

  if (is_pan){
    if (ang_value >= 1.2217) ang_value = 1.2217;            //70 deg
    else if (ang_value <= -1.2217) ang_value = -1.2217;     //-70 deg
    write_msg_.name.push_back("head_pan");
    write_msg_.position.push_back(ang_value);
  }else{
    if (ang_value >= 0.34906) ang_value = 0.34906;        //20 deg
    else if (ang_value <= -1.2217) ang_value = -1.2217;   //-70 deg
    write_msg_.name.push_back("head_tilt");
    write_msg_.position.push_back(ang_value);
  }
  write_joint_pub_.publish(write_msg_);
}

void Calculatedistance(){
  double distance = CAMERA_HEIGHT_ * tan(M_PI * 0.5 + head_tilt_ - hip_pitch_offset_);
  red_distance.publish(distance);
}


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
  red_distance = nh.advertise<std_msgs::Int32>("/robotis0/red_distance", 0);
  red_loc = nh.subscribe("/robotis0/red_cube_loc",10, &locCallback);
  read_joint_sub_ = nh.subscribe("/robotis_" + std::to_string(robot_id) + "/present_joint_states", 10, &jointStatesCallback);


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

  controller->setCtrlModule("direct_control_module");
  usleep(1000 * 1000);

  // std_msgs::String walking_msg;
  // walking_msg.data = "start";

  // g_walking_pub.publish(walking_msg);
  // ROS_INFO("Start walking");

  usleep(100 * 1000);

  while (ros::ok())
  {
    usleep(100 * 1000);
    //g_walking_pub.publish(walking_msg);
    Center();
    Calculatedistance();

    
    ros::spinOnce();
  }

  return 0;
}