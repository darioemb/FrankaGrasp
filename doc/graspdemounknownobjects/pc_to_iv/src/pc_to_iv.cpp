#include <stdio.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <sstream>

//PCL includes
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/ros/conversions.h"
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include "iv_io.h"

//#include <opencv2/highgui.h>

//Ros includes
#include <ros/ros.h>
//#include "pcl_ros/publisher.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/String.h"

#define BUFSIZE 512

using namespace pcl;
using namespace pcl::io;
using namespace std;

int meshcnt; //mesh count
ros::Publisher iv_filename_pub;

void generateInventor(const sensor_msgs::PointCloud2::ConstPtr& msg){


  ROS_INFO("Generating iv-file");
  ROS_INFO(" - Read point cloud");
  PointCloud<PointXYZ>::Ptr pc (new PointCloud<PointXYZ>);
  fromROSMsg(*msg,*pc);

  ROS_INFO(" - Convert point cloud");
  

  // Normal estimation
  NormalEstimation<PointXYZ, Normal> n;
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal>);
  KdTree<PointXYZ>::Ptr tree (new KdTreeFLANN<PointXYZ>);
  tree->setInputCloud (pc);
  n.setInputCloud (pc);
  n.setSearchMethod (tree);
  n.setKSearch (30);
  n.compute (*normals);
  //cout << *normals;

  // Concatenate the XYZ and normal fields
  PointCloud<PointNormal>::Ptr pc_with_normals (new PointCloud<PointNormal>);
  pcl::concatenateFields (*pc, *normals, *pc_with_normals);

  // Create search tree
  KdTree<PointNormal>::Ptr tree2 (new KdTreeFLANN<PointNormal>);
  tree2->setInputCloud (pc_with_normals);

  // Initialize objects
  GreedyProjectionTriangulation<PointNormal> gp3;
  PolygonMesh triangles;

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius (0.065); 	//default 0.025

  // Set typical values for the parameters
  gp3.setMu (3.5); 		//default 2.5
  gp3.setMaximumNearestNeighbors (150); //default 100
  gp3.setMaximumSurfaceAgle(M_PI/2); // default M_PI/4 => 45 degrees
  gp3.setMinimumAngle(M_PI/36); // default M_PI/18 => 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(false); //default: true

  // Get result
  gp3.setInputCloud (pc_with_normals);
  gp3.setSearchMethod (tree2);
  gp3.reconstruct (triangles);
  cout << *pc_with_normals << endl;

  //saveVTKFile ("mesh.vtk", triangles);
  ROS_INFO(" Save mesh_i.iv in /tmp/" );
  //ROS_INFO(" Save mesh.iv in ~/.ros/(data) (roslaunch) or in pc_to_iv/(data) (rosrun) " );

  //cpp int to string conversion and publish iv name

  string s;
  stringstream ss;
  std_msgs::String ivfilename;
  ss << ++meshcnt;
  s = ss.str();
  s = "/tmp/mesh_"+ s +".iv"; //name of new iv file
  ivfilename.data = s;

  //save iv File
  saveIVFile (s, triangles);
  cout << "File " << s << " saved \n";
  //publish filename of new iv file

  iv_filename_pub.publish(ivfilename);

  return;
}

void setMeshcnt(const std_msgs::StringConstPtr& str){
  meshcnt = 0;
}

int main(int argc, char **argv){

  //Subscribe to point cloud topic
  meshcnt = 0;
  ros::init (argc, argv, "pc_to_iv");
  ros::NodeHandle nh;
  iv_filename_pub = nh.advertise<std_msgs::String>("/pc_to_iv/generated_ivfilename",10);
  //ros::Duration(0.3).sleep(); //needed?

  ros::Subscriber sub = nh.subscribe("/SS/points2_without_basket_wcs", 10, generateInventor);
  ros::Subscriber meshcnt_sub = nh.subscribe("SS/doSingleShot", 1, setMeshcnt);
  ROS_INFO("Starting closed mesh generator");
  ros::spin();
  return 0;
}
