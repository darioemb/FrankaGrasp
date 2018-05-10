#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include<geometric_shapes/solid_primitive_dims.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/GraspPlanning.h>

#include <agile_grasp/Grasps.h>
#include <agile_grasp/Grasp.h>

#include<moveit_msgs/CollisionObject.h>
#include <ros/ros.h>

#include <std_msgs/Bool.h>

#include<iostream>

geometry_msgs::PoseStamped home;
geometry_msgs::PoseStamped ready;
geometry_msgs::PoseStamped pre;

geometry_msgs::PoseStamped offset;

agile_grasp::Grasps grasps;

ros::Publisher pub_off;
ros::Publisher pub_;
ros::Subscriber sub_;

const double torus_width = 0.014;

static const std::string PLANNING_GROUP = "panda_arm";
static const std::string PLANNING_EEF = "panda_hand";
moveit::planning_interface::MoveGroupInterface* move_group;
moveit::planning_interface::MoveGroupInterface* move_eef;

int computeBestGrasp(const agile_grasp::Grasps& msg){return 0;}


void cmd_Gripper(const double panda_finger_joint1, const double panda_finger_joint2)
{
  std::vector<std::string> joint_names = move_eef->getJointNames();
  move_eef->setJointValueTarget(joint_names[1], panda_finger_joint1);
  move_eef->setJointValueTarget(joint_names[0], panda_finger_joint2);
  move_eef->move();
}

//------------------------------------------------------------------------------

void pick()
{
  auto point = grasps.grasps[computeBestGrasp(grasps)];
  offset.pose.position.x=-point.surface_center.x-0.014;
  offset.pose.position.y=-point.surface_center.y-0.014;
  offset.pose.position.z=-point.surface_center.z;
  geometry_msgs::Pose agile_pose;
  agile_pose.position.x = point.surface_center.x + 0.5;
  agile_pose.position.y = point.surface_center.y;
  agile_pose.position.z = pre.pose.position.z;
  agile_pose.orientation.x = 0.923955;
  agile_pose.orientation.y = -0.382501;
  agile_pose.orientation.z = -0.00045;
  agile_pose.orientation.w = 0.000024;
  move_group->setPoseTarget(agile_pose);
  move_group->move();
  agile_pose.position.z = point.surface_center.z + 0.1;
  move_group->setPoseTarget(agile_pose);
  move_group->move();
  pub_off.publish(offset.pose);
  cmd_Gripper(torus_width,torus_width);
  std_msgs::Bool f;
  f.data=true;
  pub_.publish(f);
}

void place()
{
  geometry_msgs::PoseStamped pioloTarget;
  pioloTarget.header.frame_id="/panda_link0";
  pioloTarget.pose.position.x = 0.5-offset.pose.position.x;
  pioloTarget.pose.position.y = 0.25-offset.pose.position.y;
  pioloTarget.pose.position.z = 0.20-offset.pose.position.z;
  pioloTarget.pose.orientation.x = 0.923955;
  pioloTarget.pose.orientation.y = -0.382501;
  pioloTarget.pose.orientation.z = -0.00045;
  pioloTarget.pose.orientation.w = 0.000024;
  move_group->setPoseTarget(pioloTarget);
  move_group->move();
  cmd_Gripper(0.03, 0.03);
  std_msgs::Bool f;
  f.data=false;
  pub_.publish(f);
}

//------------------------------------------------------------------------------
void grasp_callback(const agile_grasp::GraspsConstPtr& msg) 
{
  grasps=*msg;
  // pre-posture on piolo
  pre.header.frame_id="/panda_link0";
  pre.pose.position.x = 0.5;
  pre.pose.position.y = 0;
  pre.pose.position.z = 0.20;
  pre.pose.orientation.x = 0.923955;
  pre.pose.orientation.y = -0.382501;
  pre.pose.orientation.z = -0.000045;
  pre.pose.orientation.w = 0.000024;
  move_group->setPoseTarget(pre);
  move_group->move();

  cmd_Gripper(0.03, 0.03);
  pick();
  move_group->setPoseTarget(pre);
  move_group->move();
  place();
  cmd_Gripper(0.01,0.01);
  move_group->setPoseTarget(home);
  move_group->move();
}

void init()
{
  move_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
  move_eef = new moveit::planning_interface::MoveGroupInterface(PLANNING_EEF);

 
  home.header.frame_id = "/panda_link0";
  home.pose.position.x = 0.50;
  home.pose.position.y = 0.0;
  home.pose.position.z = 0.40;
  home.pose.orientation.x = 0.923955;
  home.pose.orientation.y = -0.382501;
  home.pose.orientation.z = -0.00045;
  home.pose.orientation.w = 0.000024;


  // robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  // robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
  // planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
  // robot_state::RobotState& robot_state = planning_scene->getCurrentStateNonConst();
  // std::cout<<robot_state<<"\n";
  // robot_state::RobotState goal_state(robot_model);

}

void end()
{
  delete move_group;
  delete move_eef;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "agile_pick_place");

  ros::NodeHandle nh_("~");

  sub_ = nh_.subscribe("/find_grasps/grasps", 1, grasp_callback);
  pub_ = nh_.advertise<std_msgs::Bool>("/attached", 1);
  pub_off = nh_.advertise<geometry_msgs::Pose>("/offset_grasp", 1);

  
  ros::AsyncSpinner spinner(2);
  init();  
  spinner.start();

  move_group->setPoseTarget(home);
  move_group->move();
  
  ros::waitForShutdown();

  end();

  return 0;
}
