namespace pick_place
{
template <typename T>
PP<T>::PP(ros::NodeHandle &nh_)
{
  move_group = new m_plan_group(PLANNING_GROUP);
  move_eef = new m_plan_group(PLANNING_EEF);

  sub_grasps_points =
      nh_.subscribe("/find_grasps/grasps", 1, &PP<T>::grasps_points_Callback, this);
  sub_pose_obj = nh_.subscribe("/pose_obj", 1, &PP<T>::pose_obj_Callback, this);
  sub_from_to = nh_.subscribe("/from_to", 1, &PP<T>::from_to_Callback, this);

  pub_attach_obj = nh_.advertise<std_msgs::Bool>("/attached", 1);
  pub_offset_obj = nh_.advertise<geometry_msgs::PoseStamped>("/offset_grasp", 1);

  init();
  reset();
};

template <typename T>
PP<T>::~PP()
{
  delete move_group;
  delete move_eef;
};

template <typename T>
void PP<T>::init()
{
  home.header.frame_id = "/panda_link0";
  home.pose.position.x = 0.30;
  home.pose.position.y = 0.00;
  home.pose.position.z = 0.40;
  home.pose.orientation.x = 0.923955;
  home.pose.orientation.y = -0.382501;
  home.pose.orientation.z = -0.00045;
  home.pose.orientation.w = 0.000024;
};

template <typename T>
void PP<T>::reset()
{
  pre_piolo[0].header.frame_id = "/panda_link0";
  pre_piolo[0].pose.position.x = 0.50;
  pre_piolo[0].pose.position.y = 0.00;
  pre_piolo[0].pose.position.z = 0.20;
  pre_piolo[0].pose.orientation.x = 0.923955;
  pre_piolo[0].pose.orientation.y = -0.382501;
  pre_piolo[0].pose.orientation.z = -0.00045;
  pre_piolo[0].pose.orientation.w = 0.000024;

  pre_piolo[1].header.frame_id = "/panda_link0";
  pre_piolo[1].pose.position.x = 0.50;
  pre_piolo[1].pose.position.y = -0.25;
  pre_piolo[1].pose.position.z = 0.20;
  pre_piolo[1].pose.orientation.x = 0.923955;
  pre_piolo[1].pose.orientation.y = -0.382501;
  pre_piolo[1].pose.orientation.z = -0.00045;
  pre_piolo[1].pose.orientation.w = 0.000024;

  pre_piolo[2].header.frame_id = "/panda_link0";
  pre_piolo[2].pose.position.x = 0.50;
  pre_piolo[2].pose.position.y = 0.25;
  pre_piolo[2].pose.position.z = 0.20;
  pre_piolo[2].pose.orientation.x = 0.923955;
  pre_piolo[2].pose.orientation.y = -0.382501;
  pre_piolo[2].pose.orientation.z = -0.00045;
  pre_piolo[2].pose.orientation.w = 0.000024;
};

template <typename T>
void PP<T>::place()
{
  cmd_Gripper(0.03, 0.03);
  flag.data=false;
  pub_attach_obj.publish(flag);
};

template <typename T>
void PP<T>::pick()
{
  auto point = grasp_point_msg.result;

  offset.pose.position.x = -point.graspOutput.averagedGraspPoint.x;
  offset.pose.position.y = -point.graspOutput.averagedGraspPoint.y;
  offset.pose.position.z = -point.graspOutput.averagedGraspPoint.z;

  geometry_msgs::PoseStamped haf_avg;
  haf_avg.header.frame_id = "/panda_link0";
  haf_avg.pose.position.x = point.graspOutput.averagedGraspPoint.x - pose_obj.pose.position.x;
  haf_avg.pose.position.y = point.graspOutput.averagedGraspPoint.y + pose_obj.pose.position.y;
  haf_avg.pose.position.z = point.graspOutput.averagedGraspPoint.z + 0.1;
  haf_avg.pose.orientation.x = 0.923955;
  haf_avg.pose.orientation.y = -0.382501;
  haf_avg.pose.orientation.z = -0.00045;
  haf_avg.pose.orientation.w = 0.000024;
  move_group->setPoseTarget(haf_avg);
  move_group->move();

  pub_offset_obj.publish(offset);

  cmd_Gripper(0.014, 0.014);
  flag.data=true;
  pub_attach_obj.publish(flag);
};

template <typename T>
void PP<T>::cmd_Gripper(const double panda_finger_joint1, const double panda_finger_joint2)
{
  std::vector<std::string> joint_names = move_eef->getJointNames();
  move_eef->setJointValueTarget(joint_names[1], panda_finger_joint1);
  move_eef->setJointValueTarget(joint_names[0], panda_finger_joint2);
  move_eef->move();
};

template <typename T>
void PP<T>::grasps_points_Callback(const T &msg)
{
  grasp_point_msg = msg;
};

template <typename T>
void PP<T>::pose_obj_Callback(const geometry_msgs::PoseStampedConstPtr &msg)
{
  pose_obj = *msg;
};

template <typename T>
void PP<T>::from_to_Callback(const std_msgs::Int16MultiArrayConstPtr &msg)
{
  move_group->setPoseTarget(home);
  move_group->move();
  grasp_Execution(msg->data.at(0), msg->data.at(1));
  move_group->setPoseTarget(home);
  move_group->move();
};

template <typename T>
void PP<T>::grasp_Execution(int from, int to)
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

  pre_piolo[to].pose.position.z = 0.15;  // place of obj
  move_group->setPoseTarget(pre_piolo[to]);
  move_group->move();

  place();

  cmd_Gripper(0.01, 0.01);
  move_group->setPoseTarget(home);
  move_group->move();
  reset();
};

template <typename T>
int PP<T>::compute_best_GraspPoint(){};

}  // namespace pick_place