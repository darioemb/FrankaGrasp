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
#include <std_msgs/Int16MultiArray.h>

#include<iostream>

geometry_msgs::PoseStamped home;
geometry_msgs::PoseStamped pre_piolo[3];
geometry_msgs::PoseStamped offset;
geometry_msgs::PoseStamped obj_pose;

agile_grasp::Grasps grasps;

ros::Publisher pubOff;
ros::Publisher pub_;
ros::Subscriber subGrasp_;
ros::Subscriber sub_obj_pose_;
ros::Subscriber sub_from_to;


const double torus_width = 0.014;

static const std::string PLANNING_GROUP = "panda_arm";
static const std::string PLANNING_EEF = "panda_hand";
moveit::planning_interface::MoveGroupInterface* move_group;
moveit::planning_interface::MoveGroupInterface* move_eef;

void grasp_execution(int from, int to);
void reset();

//------------------------------------------------------------------------------
void grasp_callback(const agile_grasp::GraspsConstPtr& msg) 
{
  grasps=*msg;
}

void poseObj_callback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  obj_pose = *msg;
}

void from_to_callback(const std_msgs::Int16MultiArrayConstPtr& msg)
{
  grasp_execution(msg->data.at(0),msg->data.at(1));
}


int computeBestGrasp()
{
  int index = 0;
  int i = 0;
  int min_so_far = 100;
  for(auto x : grasps.grasps)
  {
    if(min_so_far<std::abs(x.surface_center.x))
    {
      index = i;
      min_so_far = x.surface_center.x;
    }
    ++i;
  }
  std::cout<<"--------------------\nBest grasp:\n"<<grasps.grasps[index]<<"\n--------------------\n";
  return index;
}


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
  auto point = grasps.grasps[computeBestGrasp()];

  //Set offset position TODO: Guarda meglio
  offset.pose.position.y=-point.surface_center.y;//-torus_width;
  offset.pose.position.x=-point.surface_center.x;//-torus_width;
  offset.pose.position.z=-point.surface_center.z;

  //Agile pose
  geometry_msgs::Pose agile_pose;
  agile_pose.position.x = point.surface_center.x - obj_pose.pose.position.x;
  agile_pose.position.y = point.surface_center.y +  obj_pose.pose.position.y;
  agile_pose.position.z = point.surface_center.z + 0.1;
  agile_pose.orientation.x = 0.923955;
  agile_pose.orientation.y = -0.382501;
  agile_pose.orientation.z = -0.00045;
  agile_pose.orientation.w = 0.000024;

  move_group->setPoseTarget(agile_pose);
  move_group->move();

  //Publish offset to attach
  pubOff.publish(offset.pose);

  //close gripper
  cmd_Gripper(torus_width,torus_width);

  //Attach object
  std_msgs::Bool f;
  f.data=true;
  pub_.publish(f);
}

void place()
{
  cmd_Gripper(0.03, 0.03);
  std_msgs::Bool f;
  f.data=false;
  pub_.publish(f);
}

void grasp_execution(int from, int to)
{
  move_group->setPoseTarget(pre_piolo[from]);
  move_group->move();

  cmd_Gripper(0.03, 0.03);
  
  pick();

  move_group->setPoseTarget(pre_piolo[from]);
  move_group->move();

  pre_piolo[to].pose.position.x -= offset.pose.position.x;
  pre_piolo[to].pose.position.y -= offset.pose.position.y;

  move_group->setPoseTarget(pre_piolo[to]);
  move_group->move();

  pre_piolo[to].pose.position.z = 0.15;
  move_group->setPoseTarget(pre_piolo[to]);
  move_group->move();

  place();

  cmd_Gripper(0.01,0.01);
  move_group->setPoseTarget(home);
  move_group->move();

  reset();
}

void reset()
{
  home.header.frame_id = "/panda_link0";
  home.pose.position.x = 0.50;
  home.pose.position.y = 0.0;
  home.pose.position.z = 0.40;
  home.pose.orientation.x = 0.923955;
  home.pose.orientation.y = -0.382501;
  home.pose.orientation.z = -0.00045;
  home.pose.orientation.w = 0.000024;

  pre_piolo[0].header.frame_id = "/panda_link0";
  pre_piolo[0].pose.position.x = 0.5;
  pre_piolo[0].pose.position.y = 0;
  pre_piolo[0].pose.position.z = 0.20;
  pre_piolo[0].pose.orientation.x = 0.923955;
  pre_piolo[0].pose.orientation.y = -0.382501;
  pre_piolo[0].pose.orientation.z = -0.000045;
  pre_piolo[0].pose.orientation.w = 0.000024;

  pre_piolo[1].header.frame_id = "/panda_link0";
  pre_piolo[1].pose.position.x = 0.5;
  pre_piolo[1].pose.position.y = -0.25;
  pre_piolo[1].pose.position.z = 0.20;
  pre_piolo[1].pose.orientation.x = 0.923955;
  pre_piolo[1].pose.orientation.y = -0.382501;
  pre_piolo[1].pose.orientation.z = -0.000045;
  pre_piolo[1].pose.orientation.w = 0.000024;

  pre_piolo[2].header.frame_id = "/panda_link0";
  pre_piolo[2].pose.position.x = 0.5;
  pre_piolo[2].pose.position.y = 0.25;
  pre_piolo[2].pose.position.z = 0.20;
  pre_piolo[2].pose.orientation.x = 0.923955;
  pre_piolo[2].pose.orientation.y = -0.382501;
  pre_piolo[2].pose.orientation.z = -0.000045;
  pre_piolo[2].pose.orientation.w = 0.000024;
}

void init()
{
  move_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
  move_eef = new moveit::planning_interface::MoveGroupInterface(PLANNING_EEF);


  reset();
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

  subGrasp_ = nh_.subscribe("/find_grasps/grasps", 1, grasp_callback);
  pub_ = nh_.advertise<std_msgs::Bool>("/attached", 1);
  pubOff = nh_.advertise<geometry_msgs::Pose>("/offset_grasp", 1);
  sub_obj_pose_ = nh_.subscribe("/pose_obj",1,poseObj_callback);
  sub_from_to = nh_.subscribe("/from_to",1,from_to_callback);

  
  ros::AsyncSpinner spinner(2);
  init();  
  spinner.start();

  move_group->setPoseTarget(home);
  move_group->move();
  
  ros::waitForShutdown();

  end();

  return 0;
}
