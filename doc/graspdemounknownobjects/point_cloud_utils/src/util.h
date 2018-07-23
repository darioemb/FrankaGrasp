#ifndef POINT_CLOUD_UTILS_UTIL_H
#define POINT_CLOUD_UTILS_UTIL_H

#include <iostream>
#include <fstream>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <stdlib.h>
#include <time.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>

#include "pcl/io/io.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"

#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/filters/statistical_outlier_removal.h"  // to filter outliers
#include "pcl/filters/voxel_grid.h" // for downsampling the point cloud
#include "pcl/filters/passthrough.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/features/normal_3d.h"
#include "pcl/kdtree/kdtree.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/filters/project_inliers.h"
#include "pcl/surface/convex_hull.h"
#include "pcl/segmentation/extract_polygonal_prism_data.h"
#include "pcl/segmentation/extract_clusters.h"

#include <dynamic_reconfigure/server.h>
#include <point_cloud_utils/utilConfig.h>

#include "adept_server/AnglesRequest.h"
#include "arm_geometry/TFMatRequest.h"

using namespace std;

typedef struct
{
  float x, y, z;
  unsigned int index;
} point;

typedef struct
{
  // ax+by+cz+d = 0
  float a, b, c, d;
} plane;

typedef pcl::PointXYZ Point;
typedef pcl::KdTree < Point >::Ptr KdTreePtr;

static sensor_msgs::PointCloud2 pcd;
static sensor_msgs::PointCloud2 pcdorig;
static sensor_msgs::ImageConstPtr image;
static sensor_msgs::ImageConstPtr depth_image;
static sensor_msgs::CvBridge bridge;
static unsigned int xOffset, yOffset, zOffset, rgbOffset, indexOffset;
int RESOLUTION_X = 1024, RESOLUTION_Y = 768;

// Reconfigurable parameters
static float ransac_tol = 0.025;
static int ransac_iter = 10;
static bool run = false;
static bool reconstruct = true;
static bool clean_missing_points = false;
static bool debug = false, remove_plane = true, remove_outlier = false;
static bool write_next = false, write_binary = false;
static bool write_depth_map = false;
static int write_ascii_precision = 12;
static string write_file_prefix = "pcd";
static int write_file_num = 0;
static int outlier_mean_k = 50;
static float outlier_StddevMulThresh = 1.0;
static plane user_plane;
static bool remove_user_plane = false;
static bool transform_to_global = false;
static bool estimate_normals = false;
static int normals_k_neighbors = 20;
// Copies input to output
void clone_PointCloud2(const sensor_msgs::PointCloud2 & input,
                       sensor_msgs::PointCloud2 & output);

// Filters using a plane.  
// Removes points with ax + by + cz + d < 0
static void plane_filter(sensor_msgs::PointCloud2 & pc, plane p);

// Removes points indexed by ind
static void point_remove_by_index(sensor_msgs::PointCloud2 & pc,
                                  pcl::PointIndices ind);

// Retrieves points indexed by ind
static void point_retrieve_by_index(sensor_msgs::PointCloud2 & pc,
                                    pcl::PointIndices ind);

// Removes the largest "horizontal" plane in the point cloud
static void plane_removal(sensor_msgs::PointCloud2 & pc);

// Removes points with z val < maxZ - eps
static void clean_missing_point(sensor_msgs::PointCloud2 & pc);

/** \brief Clones and adds an Index field to the point cloud message
  * \param input the message in the sensor_msgs::PointCloud2 format
  * \param output the resultant message in the sensor_msgs::PointCloud2 format
  */
bool cloneAndAddIndexField(const sensor_msgs::PointCloud2 & input,
                           sensor_msgs::PointCloud2 & output);

// Simply uses pcl::io library to save the point cloud to a file
static void write_point_cloud(const char *fname,
                              const sensor_msgs::PointCloud2 & pc,
                              const sensor_msgs::PointCloud2 & data,
                              bool binary_mode = false);

// Tries to find the set of points in pc1 but not in pc2.
// pc2 must be a strict subset of pc1
static void point_cloud_difference(sensor_msgs::PointCloud2 & pc1,
                                   sensor_msgs::PointCloud2 & pc2,
                                   sensor_msgs::PointCloud2 & output);

// Removes statistical outliers
static void remove_statistical_outliers(sensor_msgs::PointCloud2 & pc);

// Transforms input cloud to global frame and write.  Does not overwrite in.
// Requires that adept_server/adept_server and arm_geometry/tf_mat_srv be
// online
static void transform_to_global_frame(sensor_msgs::PointCloud2 & in,
                                      sensor_msgs::PointCloud2 & out);

#endif /* POINT_CLOUD_UTILS_UTIL_H */
