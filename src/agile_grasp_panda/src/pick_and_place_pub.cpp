#include "agile_grasp_panda/pick_place.hpp"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometric_shapes/solid_primitive_dims.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/GraspPlanning.h>

#include <agile_grasp/Grasps.h>
#include <agile_grasp/Grasp.h>

#include <moveit_msgs/CollisionObject.h>
#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Int16MultiArray.h>

#include <iostream>

namespace pick_place
{
template<>
void PP<agile_grasp::GraspsConstPtr>::compute_best_GraspPoint()
{
  auto point = grasp_point_msg->grasps[0];

  double alphax = atan(point.surface_center.y/point.surface_center.x);
  double alphay = atan(point.surface_center.x/point.surface_center.y);

  double dx = -cos(alphax)*K_obj_width;
  double dy = -sin(alphay)*K_obj_width;

  best_gp.header.frame_id = FRAME_ID;
  best_gp.pose.position.x = point.surface_center.x+dx;
  best_gp.pose.position.y = point.surface_center.y+dy;
  best_gp.pose.position.z = point.surface_center.z;
  best_gp.pose.orientation.x = K_orientation_x;
  best_gp.pose.orientation.y = K_orientation_y;
  best_gp.pose.orientation.z = K_orientation_z;
  best_gp.pose.orientation.w = K_orientation_w;

  ROS_INFO("Found BGP");
  std::cout << best_gp << "\n";
}
} // namespace pick_place

int main(int argc, char **argv)
{
  ros::init(argc, argv, "agile_pick_place");

  ros::NodeHandle nh_("~");

  pick_place::PP<agile_grasp::GraspsConstPtr> pp(nh_, "/find_grasps/grasps");

  ros::AsyncSpinner spinner(2);

  spinner.start();

  ros::waitForShutdown();

  return 0;
}
