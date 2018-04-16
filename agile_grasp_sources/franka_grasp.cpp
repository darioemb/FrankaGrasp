#include<agile_grasp/panda_localizer.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "franka_bridge_node");

  ros::NodeHandle nh("~");



  PandaLocalizer::Parameters params;
  PandaLocalizer pl("merda",params);
  //ros::Subscriber input_PC2 = nh.subscribe("/input_pcd", 1, &cloud2_callback);
  pl.subscribe("/input_pcd", nh);


  //ROS_INFO("Initialization complete!");

   //pub = nh.advertise<PointCloud> ("output", 1);

   ros::spin();
}