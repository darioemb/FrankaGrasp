namespace pick_place
{
template <typename T>
PP<T>::PP(ros::NodeHandle &nh_, std::string TOPIC)
{
  move_group = new m_plan_group(PLANNING_GROUP);
  move_eef = new m_plan_group(PLANNING_EEF);

  sub_grasps_points = nh_.subscribe(TOPIC, 1, &PP<T>::grasps_points_Callback, this);
  sub_pose_obj = nh_.subscribe("/pose_obj", 1, &PP<T>::pose_obj_Callback, this);
  sub_from_to = nh_.subscribe("/from_to", 1, &PP<T>::from_to_Callback, this);

  pub_attach_obj = nh_.advertise<std_msgs::Bool>("/attached", 1);
  pub_offset_obj = nh_.advertise<geometry_msgs::PoseStamped>("/offset_grasp", 1);
  pub_from_to = nh_.advertise<std_msgs::Int16MultiArray>("from_to", 1);

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
  home.header.frame_id = FRAME_ID;
  home.pose.position.x = 0.30;
  home.pose.position.y = 0.00;
  home.pose.position.z = 0.40;
  home.pose.orientation.x = K_orientation_x;
  home.pose.orientation.y = K_orientation_y;
  home.pose.orientation.z = K_orientation_z;
  home.pose.orientation.w = K_orientation_w;
};

template <typename T>
void PP<T>::reset()
{
  pre_piolo[0].header.frame_id = FRAME_ID;
  pre_piolo[0].pose.position.x = 0.50;
  pre_piolo[0].pose.position.y = 0.00;
  pre_piolo[0].pose.position.z = 0.20;
  pre_piolo[0].pose.orientation.x = K_orientation_x;
  pre_piolo[0].pose.orientation.y = K_orientation_y;
  pre_piolo[0].pose.orientation.z = K_orientation_z;
  pre_piolo[0].pose.orientation.w = K_orientation_w;

  pre_piolo[1].header.frame_id = FRAME_ID;
  pre_piolo[1].pose.position.x = 0.50;
  pre_piolo[1].pose.position.y = -0.25;
  pre_piolo[1].pose.position.z = 0.20;
  pre_piolo[1].pose.orientation.x = K_orientation_x;
  pre_piolo[1].pose.orientation.y = K_orientation_y;
  pre_piolo[1].pose.orientation.z = K_orientation_z;
  pre_piolo[1].pose.orientation.w = K_orientation_w;

  pre_piolo[2].header.frame_id = FRAME_ID;
  pre_piolo[2].pose.position.x = 0.50;
  pre_piolo[2].pose.position.y = 0.25;
  pre_piolo[2].pose.position.z = 0.20;
  pre_piolo[2].pose.orientation.x = K_orientation_x;
  pre_piolo[2].pose.orientation.y = K_orientation_y;
  pre_piolo[2].pose.orientation.z = K_orientation_z;
  pre_piolo[2].pose.orientation.w = K_orientation_w;

  id_pick_obj = -3;
};

template <typename T>
void PP<T>::place()
{
  cmd_Gripper(K_open_eef, K_open_eef);
  flag.data = false;
  pub_attach_obj.publish(flag);
};

template <typename T>
void PP<T>::move(const geometry_msgs::PoseStamped point)
{
  move_group->setPoseTarget(point);
  move_group->move();
}

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
  move(home);
  grasp_Execution(msg->data.at(0), msg->data.at(1)); //<------------------------
  move(home);
};

template <typename T>
void PP<T>::grasp_Execution(const int from, const int to)
{
  id_pick_obj = from;
  move(pre_piolo[from]);

  cmd_Gripper(K_open_eef, K_open_eef);
  pick();

  // move(pre_piolo[from]);

  best_gp.pose.position.z = 0.20;

  move(best_gp);

  pre_piolo[to].pose.position.x -= offset.pose.position.x;
  pre_piolo[to].pose.position.y -= offset.pose.position.y;
  move(pre_piolo[to]);

  pre_piolo[to].pose.position.z = K_pre_place_z; // place of obj
  move(pre_piolo[to]);

  place();
  cmd_Gripper(K_close_eef, K_close_eef);
  move(home);

  reset();
};

template <typename T>
void PP<T>::pick()
{
  // sarebbe da calcolarsi il best grasp rispetto la posizione del piolo
  // TODO:
  // - inserire i punti di grasp in un array
  // - per poter ottimizzare il compute best grasp
  compute_best_GraspPoint();

  offset.pose.position.x = -best_gp.pose.position.x;
  offset.pose.position.y = -best_gp.pose.position.y;
  offset.pose.position.z = -best_gp.pose.position.z;
  pub_offset_obj.publish(offset);

  best_gp.pose.position.x -= pose_obj.pose.position.x;
  best_gp.pose.position.y += pose_obj.pose.position.y;
  best_gp.pose.position.z += K_height_fingers;
  move(best_gp);

  cmd_Gripper(K_obj_width, K_obj_width);
  flag.data = true;
  pub_attach_obj.publish(flag);
}

//---------------------------------[SPECIAL]------------------------------------

template <typename T>
void PP<T>::compute_best_GraspPoint(){};

} // namespace pick_place