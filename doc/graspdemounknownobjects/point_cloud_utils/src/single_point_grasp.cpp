/*********************************************************************
Publishes the corresponding point cloud point (x, y, z) point from
a (x, y) image point to the topic: "image_grasp"

*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <cv_bridge/CvBridge.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Point32.h>

#include <boost/thread.hpp>
#include <boost/format.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ros/conversions.h>

#include <iostream>
#include <fstream>
#include <cstdio>

boost::mutex sp_pcd_mutex;
static pcl::PointCloud<pcl::PointXYZ> sp_pcd;

int xImage, yImage;


static void point_cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& data)
{    
    boost::lock_guard<boost::mutex> guard(sp_pcd_mutex);
    pcl::fromROSMsg(*data,sp_pcd);
/*
    if (event != CV_EVENT_LBUTTONDOWN)
      return;
    ImageView *iv = (ImageView*)param;
*/
//    geometry_msgs::Point32 msg;
    double x = sp_pcd.points[yImage*sp_pcd.width+xImage].x;
    double y = sp_pcd.points[yImage*sp_pcd.width+xImage].y;
    double z = sp_pcd.points[yImage*sp_pcd.width+xImage].z;
    printf("%lf %lf %lf\n", x, y, z);
//    iv->mousePub.publish(msg);

    return;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_grasp");
  ros::NodeHandle n;

  if(argc >= 3) {
    xImage = atoi(argv[1]);
    yImage = atoi(argv[2]);
  }
  else {
    ROS_WARN("Must pass in the (x, y) and theta image points");
  }

  if (n.resolveName("image") == "/image") {
    ROS_WARN("image_view: image has not been remapped! Typical command-line usage:\n"
             "\t$ ./image_view image:=<image topic> [transport]");
  }
  ros::Subscriber pcdSub = n.subscribe("/bumblebee2/points2", 5, point_cloud_callback);


  ros::spinOnce();
  
  return 0;
}
