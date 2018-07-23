#include "pick_place.hpp"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include<geometric_shapes/solid_primitive_dims.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/GraspPlanning.h>

#include <gpd/GraspConfig.h>
#include <gpd/GraspConfigList.h>

#include <geometry_msgs/PoseArray.h>




#include<moveit_msgs/CollisionObject.h>
#include <ros/ros.h>
#include <std_msgs/Int16MultiArray.h>

#include <std_msgs/Bool.h>

#include<iostream>


//------------------------------------------------------------------------------
namespace pick_place
{

template <>
void PP<gpd::GraspConfigList::ConstPtr>::compute_best_GraspPoint()
{
  
  auto point = grasp_point_msg->grasps[0];
  /*
  double dx = -cos(alphax)*K_obj_width;
  double dy = -sin(alphay)*K_obj_width;
  
*/
  best_gp.header.frame_id = FRAME_ID;
/*
  double r = 0.0/*M_PI/40, p = 0.0, y = -M_PI / 4; // Rotate the previous pose by -90* about Z
  tf::Quaternion q_rot(K_orientation_x, K_orientation_y, K_orientation_z, K_orientation_w), q_tmp, q_new;

q_tmp = tf::createQuaternionFromRPY(r, p, y);
q_new = q_rot * q_tmp; //from actual configuration rotate by -90* about Z

best_gp.pose.orientation.x = q_new.getAxis().getX();
best_gp.pose.orientation.y = q_new.getAxis().getY();
best_gp.pose.orientation.z = q_new.getAxis().getZ();
best_gp.pose.orientation.w = q_new.getW();
*/


  best_gp.pose.position.x = point.top.x;//-0.01;
  best_gp.pose.position.y = point.top.y-0.01;
  best_gp.pose.position.z = point.top.z;//-0.01;

  
  best_gp.pose.orientation.x = K_orientation_x;
  best_gp.pose.orientation.y = K_orientation_y;
  best_gp.pose.orientation.z = K_orientation_z;
  best_gp.pose.orientation.w = K_orientation_w;
  

   ROS_INFO("Found BGP");
  std::cout << best_gp << "\n";
  
  
  
  }
}


//------------------------------------------------------------------------------



int main(int argc, char **argv) {
  ros::init(argc, argv, "gpd_pick_place");

  ros::NodeHandle nh_("~");

  pick_place::PP<gpd::GraspConfigList::ConstPtr> pp(nh_,"/detect_grasps/clustered_grasps");
  

  ros::AsyncSpinner spinner(2);
  
  spinner.start();

  
  
  ros::waitForShutdown();

 

  return 0;
}
