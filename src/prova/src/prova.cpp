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

#include <geometry_msgs/Pose.h>
#include <haf_grasping/CalcGraspPointsServerActionResult.h>
#include <haf_grasping/GraspOutput.h>

//------------------------------------------------------------------------------

using h_svr_result = haf_grasping::CalcGraspPointsServerActionResult;
using m_plan_group = moveit::planning_interface::MoveGroupInterface;

m_plan_group *move_group;
m_plan_group *move_eef;

ros::Subscriber sub;

static const std::string PLANNING_GROUP = "panda_arm";
static const std::string PLANNING_EEF   = "panda_hand";

//------------------------------------------------------------------------------

void openGripper(const double panda_finger_joint1, const double panda_finger_joint2)
{
   ROS_INFO("set opening gripper");
  std::vector<std::string> joint_names = move_eef->getJointNames();
  move_eef->setJointValueTarget(joint_names[1], panda_finger_joint1);
  move_eef->setJointValueTarget(joint_names[0], panda_finger_joint2);
  move_eef->move();
   ROS_INFO("Opening gripper");
}

void moveToPose(const geometry_msgs::Pose &pose)
{
  ROS_INFO("moveTO!");
  geometry_msgs::PoseStamped pp;
  pp.header.frame_id="/panda_link0";
  pp.pose = pose;
  move_group->setPoseTarget(pp);
  move_group->move();
  ROS_INFO("finich moveTO!");
}

void result_callback(h_svr_result grasp)
{
  ROS_INFO("Grasp points received");

  geometry_msgs::Pose pre_grasp;

  pre_grasp.position.x = 0.0;
  pre_grasp.position.y = 0.0;
  pre_grasp.position.z = grasp.result.graspOutput.averagedGraspPoint.z + 0.2;
  pre_grasp.orientation.x = -0.923955;
  pre_grasp.orientation.y = -0.382501;
  pre_grasp.orientation.z = -0.00045;
  pre_grasp.orientation.w = 0.000024;

  moveToPose(pre_grasp);
  openGripper(0.03, 0.03);
  auto point = grasp.result;
  //   std::cout << point.graspOutput.graspPoint1;
  //   std::cout << point.graspOutput.graspPoint2;
  //   std::cout << point.graspOutput.averagedGraspPoint;
  //   std::cout << point.graspOutput.approachVector;
  //   std::cout << "roll: " << point.graspOutput.roll;
  geometry_msgs::Pose avg_pose;
  avg_pose.position.x = point.graspOutput.averagedGraspPoint.x + 0.5;
  avg_pose.position.y = point.graspOutput.averagedGraspPoint.y;
  avg_pose.position.z = point.graspOutput.averagedGraspPoint.z + 0.1;
  // avg_pose.orientation.x = point.graspOutput.approachVector.x;
  // avg_pose.orientation.y = point.graspOutput.approachVector.y;
  // avg_pose.orientation.z = point.graspOutput.approachVector.z;
  // avg_pose.orientation.w = point.graspOutput.roll;
  avg_pose.orientation.x = -0.923955;
  avg_pose.orientation.y = -0.382501;
  avg_pose.orientation.z = -0.00045;
  avg_pose.orientation.w = 0.000024;
   ROS_INFO("set grasp point");
  // avg_pose.orientation.x = 0.0;
  // avg_pose.orientation.y = 0.0;
  // avg_pose.orientation.z = 0.0;
  // avg_pose.orientation.w = 0.1;
  /*    HOME POSE:
    pose.position.x = 0.307049;
    pose.position.y = 0.000035;
    pose.position.z = -0.590206;
    pose.orientation.x = 0.923955;
    pose.orientation.y = -0.382501;
    pose.orientation.z = -0.00045;
    pose.orientation.w = 0.000024;
  */
  moveToPose(avg_pose);
  openGripper(0.0, 0.0);
   ROS_INFO("set default pose ring");
  avg_pose.position.x = 0.0;
  avg_pose.position.y = 0.0;
  avg_pose.position.z = 0.5;
   avg_pose.orientation.x = -0.923955;
  avg_pose.orientation.y = -0.382501;
  avg_pose.orientation.z = -0.00045;
  avg_pose.orientation.w = 0.000024;
  moveToPose(avg_pose);
}

//------------------------------------------------------------------------------

// group.setPlannerId("RRTConnectkConfigDefault");

int main(int argc, char **argv)
{
  ros::init(argc, argv, "prova");

  ros::NodeHandle nh_("~");
  ros::AsyncSpinner spinner(2);
  ROS_INFO("Ready!");
  move_group = new m_plan_group(PLANNING_GROUP);
  move_eef = new m_plan_group(PLANNING_EEF);

  sub = nh_.subscribe<h_svr_result>("/calc_grasppoints_svm_action_server/result", 1, result_callback);
  spinner.start();
   ROS_INFO("finish!");
  ros::waitForShutdown();
  
  delete move_group;
  delete move_eef;
  
  return 0;
}
