#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

// #include <moveit_visual_tools/moveit_visual_tools.h>

#include <std_msgs/Bool.h>

#include <ros/ros.h>
#include <iostream>

moveit::planning_interface::MoveGroupInterface* move_group;

int main(int argc, char* argv[])
{
  /* code for main function */
  ros::init(argc, argv, "prova");

  static const std::string PLANNING_GROUP = "panda_arm";

   move_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);

  //   geometry_msgs::PoseStamped random_pose = move_group->getRandomPose();
//   geometry_msgs::PoseStamped home_pos;
//   home_pos.pose.position.x = 0.307049;
//   home_pos.pose.position.y = 0.000035;
//   home_pos.pose.position.z = 0.590206;
//   home_pos.pose.orientation.x = 0.923955;
//   home_pos.pose.orientation.y = -0.382501;
//   home_pos.pose.orientation.z = -0.00145;
//   home_pos.pose.orientation.w = 0.002024;


//   move_group->setPoseTarget(home_pos);
//   move_group->move();

// const robot_state::JointModelGroup *joint_model_group =
//   move_group->getCurrentState()->getJointModelGroup(move_group->getName());
robot_state::RobotState start_state(move_group->getRobotModel());

const robot_state::JointModelGroup *joint_model_group =
                start_state.getJointModelGroup(move_group->getName());


geometry_msgs::Pose pose;
pose.position.x = 0.307049;
pose.position.y = 0.000035;
pose.position.z = 0.590206;
pose.orientation.w = 0.002024;

move_group->setPoseTarget(pose);

move_group->move();
  return 0;
}