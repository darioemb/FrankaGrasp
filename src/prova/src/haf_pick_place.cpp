#include <haf_grasping/CalcGraspPointsServerActionResult.h>
#include <haf_grasping/GraspOutput.h>

#include "haf_pick_place/haf_pick_place.hpp"

//------------------------------------------------------------------------------

using haf_result = haf_grasping::CalcGraspPointsServerActionResult;

namespace pick_place
{
template <>
void PP<haf_result>::compute_best_GraspPoint()
{
  ROS_INFO("Compute BGP");
  auto point = grasp_point_msg.result;
  gps_msg[0] = point.graspOutput.graspPoint1;
  gps_msg[1] = point.graspOutput.graspPoint2;
  gps_msg[2] = point.graspOutput.averagedGraspPoint;

  auto curr = gps_msg[2];

  geometry_msgs::Point conf[4];

  conf[0].x = curr.x;
  conf[0].y = curr.y;
  conf[0].z = curr.z;
  
  conf[1].x = -curr.x;
  conf[1].y = -curr.y;
  conf[1].z = -curr.z;
  //////////////////////
  conf[2].x = curr.y;
  conf[2].y = curr.x;
  conf[2].z = curr.z;
  //////////////////////
  conf[3].x = -curr.y;
  conf[3].y = -curr.x;
  conf[3].z = -curr.z;

  // for (auto x : conf)
  // {
  //   if (x.x > pre_piolo[id_pick_obj].pose.position.x)
  //   curr = x;
  // }
/* 
  auto point = grasp_point_msg->grasps[0];

  best_gp.header.frame_id = FRAME_ID;
  best_gp.pose.position.x = (point.surface_center.x + point.center.x) / 2;
  best_gp.pose.position.y = (point.surface_center.y + point.center.y) / 2;
  best_gp.pose.position.z = point.surface_center.z;

  // Orientation
  double r = 0.0, p = 0.0, y = -M_PI / 2; // Rotate the previous pose by -90* about Z
  tf::Quaternion q_rot(K_orientation_x, K_orientation_y, K_orientation_z, K_orientation_w), q_tmp, q_new;

  q_tmp = tf::createQuaternionFromRPY(r, p, y);
  q_new = q_rot * q_tmp; //from actual configuration rotate by -90* about Z

  best_gp.pose.orientation.x = q_new.getAxis().getX();
  best_gp.pose.orientation.y = q_new.getAxis().getY();
  best_gp.pose.orientation.z = q_new.getAxis().getZ();
  best_gp.pose.orientation.w = q_new.getW();
  */


  best_gp.header.frame_id = FRAME_ID;
  best_gp.pose.position.x =conf[2].x; //best configuration!
  best_gp.pose.position.y =conf[2].y;
  best_gp.pose.position.z =conf[2].z;
  // double r = 0.0, p = 0.0, y = -M_PI / 2; // Rotate the previous pose by -90* about Z
  // tf::Quaternion q_rot(K_orientation_x, K_orientation_y, K_orientation_z, K_orientation_w), q_tmp, q_new;

  // q_tmp = tf::createQuaternionFromRPY(r, p, y);
  // q_new = q_rot * q_tmp; //from actual configuration rotate by -90* about Z

  // best_gp.pose.orientation.x = q_new.getAxis().getX();
  // best_gp.pose.orientation.y = q_new.getAxis().getY();
  // best_gp.pose.orientation.z = q_new.getAxis().getZ();
  // best_gp.pose.orientation.w = q_new.getW();

  // pre_piolo[0].pose.orientation = best_gp.pose.orientation;
  // pre_piolo[1].pose.orientation = best_gp.pose.orientation;
  // pre_piolo[2].pose.orientation = best_gp.pose.orientation;
  best_gp.pose.orientation.x = K_orientation_x;
  best_gp.pose.orientation.y = K_orientation_y;
  best_gp.pose.orientation.z = K_orientation_z;
  best_gp.pose.orientation.w = K_orientation_w;

  ROS_INFO("Found BGP");
  std::cout << best_gp << "\n";
}

}  // namespace pick_place

//----------------------------------[ MAIN ]------------------------------------

int main(int argc, char **argv)
{
  ros::init(argc, argv, "haf_pick_place");

  ros::NodeHandle nh_("~");

  pick_place::PP<haf_result> pp(nh_, "/calc_grasppoints_svm_action_server/result");

  ros::AsyncSpinner spinner(2);

  spinner.start();

  ros::waitForShutdown();

  return 0;
}

//------------------------------------------------------------------------------