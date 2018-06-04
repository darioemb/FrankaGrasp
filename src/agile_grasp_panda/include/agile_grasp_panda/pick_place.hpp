#pragma once

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
// #include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/String.h>
#include <iostream>

namespace pick_place
{
using m_plan_group = moveit::planning_interface::MoveGroupInterface;

static const std::string PLANNING_GROUP = "panda_arm";
static const std::string PLANNING_EEF = "panda_hand";
static const std::string FRAME_ID = "/panda_link0";

const double K_orientation_x = 0.923955;
const double K_orientation_y = -0.382501;
const double K_orientation_z = -0.00045;
const double K_orientation_w = 0.000024;
const double K_obj_width = 0.014;
const double K_open_eef = 0.03;
const double K_close_eef = 0.01;
const double K_pre_place_z = 0.15;
const double K_height_fingers = 0.10;

template <typename T>
class PP
{
public:
  PP(ros::NodeHandle &nh_, const std::string TOPIC);
  ~PP();

  void init();
  void reset();

  void move(const geometry_msgs::PoseStamped point);
  void pick(); 
  void place();

  void cmd_Gripper(const double panda_finger_joint1, const double panda_finger_joint2);

  void grasps_points_Callback(const T &msg);

  void pose_obj_Callback(const geometry_msgs::PoseStampedConstPtr &msg);

  void from_to_Callback(const std_msgs::Int16MultiArrayConstPtr &msg);

  void grasp_Execution(const int from, const int to);

  void compute_best_GraspPoint();  // special

private:
  ros::Subscriber sub_grasps_points;
  ros::Subscriber sub_from_to;
  ros::Subscriber sub_pose_obj;

  ros::Publisher pub_offset_obj;
  ros::Publisher pub_attach_obj;
  ros::Publisher pub_from_to;

  m_plan_group *move_group;
  m_plan_group *move_eef;

  geometry_msgs::PoseStamped home;
  geometry_msgs::PoseStamped offset;
  geometry_msgs::PoseStamped pre_piolo[3];
  geometry_msgs::PoseStamped pose_obj;
  geometry_msgs::PoseStamped best_gp;

  geometry_msgs::Point gps_msg[3];

  std_msgs::Bool flag;

  T grasp_point_msg;  // haf_grasp

  int id_pick_obj{ -3 };
};

}  // namespace pick_place

#include "pick_place.i.hpp"