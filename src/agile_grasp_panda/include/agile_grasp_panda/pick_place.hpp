#pragma once

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16MultiArray.h>
#include <iostream>

namespace pick_place
{

using m_plan_group = moveit::planning_interface::MoveGroupInterface;

static const std::string PLANNING_GROUP = "panda_arm";
static const std::string PLANNING_EEF = "panda_hand";

const double obj_width = 0.014;

template <typename T>
class PP
{
  public:
    PP(ros::NodeHandle &nh_);
    ~PP();

    void init();
    void reset();

    void pick();
    void place();

    void cmd_Gripper(const double panda_finger_joint1, const double panda_finger_joint2);

    void grasps_points_Callback(const T &msg);

    void pose_obj_Callback(const geometry_msgs::PoseStampedConstPtr &msg);

    void from_to_Callback(const std_msgs::Int16MultiArrayConstPtr &msg);

    void grasp_Execution(int from, int to);

    int compute_best_GraspPoint();

  private:
    ros::Subscriber sub_grasps_points;
    ros::Subscriber sub_from_to;
    ros::Subscriber sub_pose_obj;

    ros::Publisher pub_offset_obj;
    ros::Publisher pub_attach_obj;

    m_plan_group *move_group;
    m_plan_group *move_eef;

    geometry_msgs::PoseStamped home;
    geometry_msgs::PoseStamped offset;
    geometry_msgs::PoseStamped pre_piolo[3];
    geometry_msgs::PoseStamped pose_obj;

    std_msgs::Bool flag;

    T grasp_point_msg; //grasp 
};

} // namespace pick_place

#include "pick_place.i.hpp" //<--- check