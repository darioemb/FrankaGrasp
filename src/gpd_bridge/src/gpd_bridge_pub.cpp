
//#include "ros/ros.h"
//#include "std_msgs/String.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <gpd/GraspConfig.h>
#include <gpd/GraspConfigList.h>

#include <ros/ros.h>
#include <iostream>

static const std::string PLANNING_GROUP = "panda_arm";
static const std::string PLANNING_GROUP = "panda_hand";

moveit::planning_interface::MoveGroupInterface* move_group;
moveit::planning_interface::MoveGroupInterface* hand_group;


void openGripper(const double panda_finger_joint1, const double panda_finger_joint2)
{
  std::vector<std::string> joint_names = hand_group->getJointNames();

  hand_group->setJointValueTarget(joint_names[1],panda_finger_joint1);
  hand_group->setJointValueTarget(joint_names[0],panda_finger_joint2);
  hand_group->move();
}

void moveToPose(const geometry_msgs::Pose &pose)
{

geometry_msgs::PoseStamped pp;
pp.header.frame_id="/panda_link0";
pp.pose=pose;
move_group->setPoseTarget(pp);
move_group->move();
}

gpd::GraspConfigList grasps;



void gpdCallback(const gpd::GraspConfigList::ConstPtr& msg)
{
	ROS_INFO("Grasp points received");
  grasps=*msg;
		
  
   

 geometry_msgs::Pose pose;
  pose.position.x = grasps.grasps[0].sample.x+0.5;
  pose.position.y = grasps.grasps[0].sample.y;
  pose.position.z = grasps.grasps[0].sample.z+0.1514;
  
  pose.orientation.x = 0.923955;
  pose.orientation.y = -0.382501;
  pose.orientation.z = -0.000045;
  pose.orientation.w = 0.000024;
  
moveToPose(pose);
  openGripper(0.025,0.025);
  pose.position.z = pose.position.z-0.12;
  moveToPose(pose);
  openGripper(0.0,0.0);
  pose.position.x=0.5;
  pose.position.y=0.0;
  pose.position.z=0.5;
  moveToPose(pose);
}

int main(int argc, char* argv[])
{

  /* code for main function */
  
  ros::init(argc, argv, "gpd_grasp_moveit_bridge");
  ros::NodeHandle nh("~");
  ros::AsyncSpinner spinner(2);
  
  move_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
  hand_group = new moveit::planning_interface::MoveGroupInterface("panda_hand");

  
  ros::Subscriber sub = nh.subscribe("/detect_grasps/clustered_grasps", 1, gpdCallback);
  
  spinner.start();
  


  ros::waitForShutdown();
  
  return 0;
}





