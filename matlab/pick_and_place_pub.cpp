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
void PP<gpd::GraspConfigList::ConstPtr>::pick()
{
  
  auto point = grasp_point_msg->grasps[0];
  
  offset.pose.position.x=-point.sample.x; 
  offset.pose.position.y=-point.sample.y;
  offset.pose.position.z=-point.sample.z;
  
 

  
  
  geometry_msgs::Pose gpd_pose;
  gpd_pose.position.x = point.sample.x - pose_obj.pose.position.x ;
  gpd_pose.position.y = point.sample.y; - pose_obj.pose.position.y ;
  gpd_pose.position.z = point.sample.z + 0.1;
  
  
  gpd_pose.orientation.x = 0.923955;
  gpd_pose.orientation.y = -0.382501;
  gpd_pose.orientation.z = -0.00045;
  gpd_pose.orientation.w = 0.000024;
  
  move_group->setPoseTarget(gpd_pose);
  move_group->move();

 
  pub_offset_obj.publish(offset);

  cmd_Gripper(obj_width, obj_width);
  
  
  flag.data=true;
  pub_attach_obj.publish(flag);
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
