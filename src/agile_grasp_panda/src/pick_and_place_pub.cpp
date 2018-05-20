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


// int computeBestGrasp()
// {
//   int index = 0;
//   int i = 0;
//   int min_so_far = 100;
//   for (auto x : grasps.grasps)
//   {
//     if (min_so_far < std::abs(x.surface_center.x))
//     {
//       index = i;
//       min_so_far = x.surface_center.x;
//     }
//     ++i;
//   }
//   std::cout << "--------------------\nBest grasp:\n"
//             << grasps.grasps[index] << "\n--------------------\n";
//   return index;
// }

namespace pick_place
{
template <>
void PP<agile_grasp::GraspsConstPtr>::pick()
{
   auto point = grasp_point_msg->grasps[0];

  offset.pose.position.y = -point.surface_center.y; //-torus_width;
  offset.pose.position.x = -point.surface_center.x; //-torus_width;
  offset.pose.position.z = -point.surface_center.z;

  geometry_msgs::Pose agile_pose;
  agile_pose.position.x = point.surface_center.x - pose_obj.pose.position.x;
  agile_pose.position.y = point.surface_center.y + pose_obj.pose.position.y;
  agile_pose.position.z = point.surface_center.z + 0.1;
  agile_pose.orientation.x = 0.923955;
  agile_pose.orientation.y = -0.382501;
  agile_pose.orientation.z = -0.00045;
  agile_pose.orientation.w = 0.000024;

  move_group->setPoseTarget(agile_pose);
  move_group->move();

  pub_offset_obj.publish(offset);

  cmd_Gripper(obj_width, obj_width);

  flag.data=true;
  pub_attach_obj.publish(flag);
}
template<>
int PP<agile_grasp::GraspConstPtr>::compute_best_GraspPoint()
{
  return 0;
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
