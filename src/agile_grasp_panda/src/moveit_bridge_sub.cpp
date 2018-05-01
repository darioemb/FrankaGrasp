#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

#include<iostream>

//torus width = 0.0014;

static const std::string PLANNING_GROUP = "panda_arm";
moveit::planning_interface::MoveGroupInterface* move_group;
moveit::planning_interface::MoveGroupInterface* hand_group;

void moveToPose(const geometry_msgs::Pose &pose)
{
  geometry_msgs::PoseStamped pp;
  pp.header.frame_id="/panda_link0";
  pp.pose=pose;
  move_group->setPoseTarget(pp);
  move_group->move();
}
std_msgs::Float64MultiArray referenceData;
void agile_graspCallback(std_msgs::Float64MultiArrayConstPtr controlDataConstPtr)
{
  referenceData = *controlDataConstPtr;
  geometry_msgs::Pose pose;
  pose.position.x = 0.5;
  pose.position.y = 0.0;
  pose.position.z = 0.5;
  pose.orientation.x = referenceData.data[0];
  pose.orientation.y = referenceData.data[1];
  pose.orientation.z = referenceData.data[2];
  pose.orientation.w = referenceData.data[3];

  moveToPose(pose);
}

int main(int argc, char* argv[])
{
  /* code for main function */
  ros::init(argc, argv, "pose_varie");
  ros::NodeHandle nh("~");
  ros::AsyncSpinner spinner(2);
  
  move_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
  hand_group = new moveit::planning_interface::MoveGroupInterface("panda_hand");

  referenceData.data.resize(4);

  ros::Subscriber sub = nh.subscribe("/muovi_pose", 1, agile_graspCallback);


  spinner.start();


  ros::waitForShutdown();

  return 0;
}