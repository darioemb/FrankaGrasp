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
template <>
void PP<agile_grasp::GraspsConstPtr>::compute_best_GraspPoint()
{

  std::ofstream fapproach("/home/sphero/code/FrankaGrasp/data_/approach.data");
  std::ofstream faxis("/home/sphero/code/FrankaGrasp/data_/axis.data");
  std::ofstream fcenter("/home/sphero/code/FrankaGrasp/data_/center.data");
  std::ofstream fsurface("/home/sphero/code/FrankaGrasp/data_/surface.data");

  ROS_INFO("Writing on files...");
  for (auto x : grasp_point_msg->grasps)
  {
    fapproach << x.approach.x << " " << x.approach.y << " " << x.approach.z << "\n";
    faxis << x.axis.x << " " << x.axis.y << " " << x.axis.z << "\n";
    fcenter << x.center.x << " " << x.center.y << " " << x.center.z << "\n";
    fsurface << x.surface_center.x << " " << x.surface_center.y << " " << x.surface_center.z << "\n";
  }
  ROS_INFO("Write ended!");

  auto point = grasp_point_msg->grasps[0];

  for(auto x : grasp_point_msg->grasps)
  {
    if(std::abs(x.surface_center.y+x.center.y)>std::abs(point.surface_center.y+point.center.y))
      point=x;
  }

  best_gp.header.frame_id = FRAME_ID;
  best_gp.pose.position.y = (point.surface_center.x + point.center.x) / 2;
  best_gp.pose.position.x = (point.surface_center.y + point.center.y) / 2;
  best_gp.pose.position.z = point.surface_center.z;

  // Orientation
  double r = 0.0/*M_PI/40*/, p = 0.0, y = M_PI / 4; // Rotate the previous pose by -90* about Z
  tf::Quaternion q_rot(K_orientation_x, K_orientation_y, K_orientation_z, K_orientation_w), q_tmp, q_new;

  q_tmp = tf::createQuaternionFromRPY(r, p, y);
  q_new = q_rot * q_tmp; //from actual configuration rotate by -90* about Z

  best_gp.pose.orientation.x = q_new.getAxis().getX();
  best_gp.pose.orientation.y = q_new.getAxis().getY();
  best_gp.pose.orientation.z = q_new.getAxis().getZ();
  best_gp.pose.orientation.w = q_new.getW();

  pre_piolo[0].pose.orientation=best_gp.pose.orientation;
  pre_piolo[1].pose.orientation=best_gp.pose.orientation;
  pre_piolo[2].pose.orientation=best_gp.pose.orientation;

  // best_gp.pose.orientation.x = K_orientation_x;
  // best_gp.pose.orientation.y = K_orientation_y;
  // best_gp.pose.orientation.z = K_orientation_z;
  // best_gp.pose.orientation.w = K_orientation_w;

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
