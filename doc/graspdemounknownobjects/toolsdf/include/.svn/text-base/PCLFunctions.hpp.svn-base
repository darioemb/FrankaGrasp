#ifndef PCLFUNCTIONS_HPP
#define PCLFUNCTIONS_HPP

#ifndef EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
 #define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
#endif

#ifndef EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 #define EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif

// PCL includes
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/pcl_base.h>
#include <pcl/pcl_macros.h>

struct PointXYZRC
{
  PCL_ADD_POINT4D;  // preferred way of adding a XYZ+padding
  int r;
  int c;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZRC,           // here we assume a XYZ + "rc" (as fields)
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (int, r, r)
                                   (int, c, c)
)

#include <pcl/filters/passthrough.h>
#include <pcl/filters/impl/passthrough.hpp>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/impl/sac_segmentation.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/impl/extract_indices.hpp>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/impl/statistical_outlier_removal.hpp>

#include <pcl/common/transform.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/impl/sac_model_plane.hpp>

#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/impl/sac_model_sphere.hpp>

#include <pcl/sample_consensus/sac_model_circle.h>
#include <pcl/sample_consensus/impl/sac_model_circle.hpp>

#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/sample_consensus/impl/sac_model_cylinder.hpp>

#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/impl/sac_model_line.hpp>

#include <pcl/sample_consensus/sac_model_parallel_line.h>
#include <pcl/sample_consensus/impl/sac_model_parallel_line.hpp>

#include <pcl/sample_consensus/sac_model_parallel_plane.h>
#include <pcl/sample_consensus/impl/sac_model_parallel_plane.hpp>

#include <pcl/sample_consensus/sac_model_normal_parallel_plane.h>
#include <pcl/sample_consensus/impl/sac_model_normal_parallel_plane.hpp>

#include <pcl/sample_consensus/sac_model_normal_plane.h>
#include <pcl/sample_consensus/impl/sac_model_normal_plane.hpp>

#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>
#include <pcl/sample_consensus/impl/sac_model_perpendicular_plane.hpp>

#include <pcl/sample_consensus/sac_model_registration.h>
#include <pcl/sample_consensus/impl/sac_model_registration.hpp>

#include <pcl/sample_consensus/lmeds.h>
#include <pcl/sample_consensus/impl/lmeds.hpp>

#include <pcl/sample_consensus/mlesac.h>
#include <pcl/sample_consensus/impl/mlesac.hpp>

#include <pcl/sample_consensus/msac.h>
#include <pcl/sample_consensus/impl/msac.hpp>

#include <pcl/sample_consensus/prosac.h>
#include <pcl/sample_consensus/impl/prosac.hpp>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/impl/ransac.hpp>

#include <pcl/sample_consensus/rmsac.h>
#include <pcl/sample_consensus/impl/rmsac.hpp>

#include <pcl/sample_consensus/rransac.h>
#include <pcl/sample_consensus/impl/rransac.hpp>

#include <stdio.h>

#include <iostream>
#include <pcl/ModelCoefficients.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/impl/normal_3d.hpp>
#include <pcl/features/principal_curvatures.h>
#include <pcl/features/impl/principal_curvatures.hpp>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

//typedef pcl::PointXYZ Point;

namespace pcl_selfmade_exceptions
{

enum PCL_SELFMADE_EXCEPTIONS
{
  PT_FILTER = 1,
  SEGMENT      ,
  NORMALS      ,
  CURVATURE    ,
};

}

template <class T>
bool FilterPointCloud(pcl::PointCloud<PointXYZRC>::ConstPtr cloud,
                      pcl::PointCloud<PointXYZRC>::Ptr &cloud_filtered)
{
  try {
    // Filter PointCloud
    pcl::PassThrough<PointXYZRC> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits(0.0, 1.0);
    pcl::PointCloud<PointXYZRC> cloud_temp;
    pass.filter(cloud_temp);

    if(!cloud_temp.size())
    {
      std::cerr << "[ERROR] After filtering not enough data." << std::endl;
      throw pcl_selfmade_exceptions::PT_FILTER;
    }

    cloud_filtered = cloud_temp.makeShared();
  }
  catch (...)
  {
    return(false);
  }
  return(true);
}

template <class T>
bool SegmentPlane(pcl::PointCloud<PointXYZRC>::ConstPtr cloud,
                  pcl::PointCloud<PointXYZRC>::Ptr &cloud_plane,
                  pcl::PointCloud<PointXYZRC>::Ptr &cloud_without_plane,
                  pcl::ModelCoefficients &coefficients)
{
  try {
    // Segment Plane
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    // Create the segmentation object
    pcl::SACSegmentation<T> seg;
    pcl::ExtractIndices<T> extract;
    // Optional
    seg.setOptimizeCoefficients (false);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, coefficients);

    if(!inliers->indices.size())
    {
      std::cerr << "[ERROR] Not enough inliers." << std::endl;
      throw pcl_selfmade_exceptions::SEGMENT;
    }

    // Create the filtering object
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    // Extract Plane
    extract.setNegative(false);
    pcl::PointCloud<T> cloud_temp;
    extract.filter(cloud_temp);

    if(!cloud_temp.size())
    {
      std::cerr << "[ERROR] Not enough points in the plane." << std::endl;
      throw pcl_selfmade_exceptions::SEGMENT;
    }

    cloud_plane = cloud_temp.makeShared();
    // Extract Points
    extract.setNegative(true);
    pcl::PointCloud<T> cloud_temp2;
    extract.filter(cloud_temp2);

    if(!cloud_temp2.size())
    {
      std::cerr << "[ERROR] Not enough points in the scene." << std::endl;
      throw pcl_selfmade_exceptions::SEGMENT;
    }

    cloud_without_plane = cloud_temp2.makeShared();
  }
  catch (...)
  {
    return(false);
  }
  return(true);
}

template <class T>
bool ComputePointNormals(pcl::PointCloud<PointXYZRC>::ConstPtr cloud,
                         pcl::PointCloud<pcl::Normal>::Ptr &cloud_normal)
{
  try {
    // create simple point cloud
    /*pcl::PointCloud<pcl::PointXYZ> temp_cloud;
    temp_cloud.points.clear();
    temp_cloud.width  = cloud->width;
    temp_cloud.height = cloud->height;
    temp_cloud.is_dense = false;
    temp_cloud.points.resize (cloud->points.size());
    for(size_t i = 0; i < cloud->points.size(); ++i)
    {
      temp_cloud.points[i].x = cloud->points[i].x;
      temp_cloud.points[i].y = cloud->points[i].y;
      temp_cloud.points[i].z = cloud->points[i].z;
    }
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr temp_cloud_ptr = temp_cloud.makeShared();*/

    // Extract point cloud normals
    pcl::NormalEstimation<PointXYZRC, pcl::Normal> ne;
    pcl::KdTreeFLANN<PointXYZRC>::Ptr tree (new pcl::KdTreeFLANN<PointXYZRC>());
    pcl::PointCloud<pcl::Normal> temp;
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud/*temp_cloud_ptr*/);
    ne.setKSearch(50);
    ne.compute(temp);

    if(!temp.size())
    {
      throw pcl_selfmade_exceptions::NORMALS;
    }

    cloud_normal = temp.makeShared();
  }
  catch (...)
  {
    return(false);
  }
  return(true);
}

template <typename T>
bool ComputePointCurvatures(pcl::PointCloud<PointXYZRC>::ConstPtr cloud,
                            pcl::PointCloud<pcl::Normal>::Ptr cloud_normal,
                            pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr &cloud_curvatures)
{
  try {
    // Extract point cloud normals
    pcl::PrincipalCurvaturesEstimation<T, pcl::Normal, pcl::PrincipalCurvatures> pce;
    pcl::KdTreeFLANN<PointXYZRC>::Ptr tree (new pcl::KdTreeFLANN<PointXYZRC>());
    pcl::PointCloud<pcl::PrincipalCurvatures> temp;
    pce.setSearchMethod(tree);
    pce.setInputNormals(cloud_normal);
    pce.setInputCloud(cloud);
    pce.setKSearch(50);
    pce.compute(temp);

    if(!temp.size())
    {
      throw pcl_selfmade_exceptions::CURVATURE;
    }

    cloud_curvatures = temp.makeShared();
  }
  catch (...)
  {
    return(false);
  }
  return(true);
}

void PointXYZ2PointXYZRC(pcl::PointCloud<pcl::PointXYZ> &temp, pcl::PointCloud<PointXYZRC> &cloud)
{
  cloud.points.clear();
  cloud.width  = temp.width;
  cloud.height = temp.height;
  cloud.is_dense = false;
  cloud.points.resize (cloud.width * cloud.height);

  for(unsigned int i = 0; i < cloud.height; ++i)
  {
    for(unsigned int j = 1; j < cloud.width; ++j)
    {
      cloud.points[j+i*cloud.width].x = temp.points[j+i*cloud.width].x;
      cloud.points[j+i*cloud.width].y = temp.points[j+i*cloud.width].y;
      cloud.points[j+i*cloud.width].z = temp.points[j+i*cloud.width].z;
      if(std::isnan(temp.points[j+i*cloud.width].x) ||
	 std::isnan(temp.points[j+i*cloud.width].y) ||
	 std::isnan(temp.points[j+i*cloud.width].z))
      {
        cloud.points[j+i*cloud.width].r = std::numeric_limits<int>::quiet_NaN();
        cloud.points[j+i*cloud.width].c = std::numeric_limits<int>::quiet_NaN();
      }
      else
      {
	cloud.points[j+i*cloud.width].r = i;
        cloud.points[j+i*cloud.width].c = j;
      }
    }
  }
}

void PointXYZ2PointXYZRC(pcl::PointCloud<pcl::PointXYZ> &p1, pcl::PointCloud<PointXYZRC> &p2,
		                           pcl::PointCloud<PointXYZRC> &cloud)
{
  if(p1.points.size() != p1.points.size())
  {
    cloud.points.clear();
    cloud.width  = (p1.points.size() < p2.points.size() ? p1.points.size() : p2.points.size());
    cloud.height = 1;
    cloud.is_dense = false;
    cloud.points.resize (cloud.width * cloud.height);
  }
  else
  {
    cloud.points.clear();
    cloud.width  = p1.width;
    cloud.height = p2.height;
    cloud.is_dense = false;
    cloud.points.resize (cloud.width * cloud.height);
  }

  for(size_t i = 0; i < cloud.points.size(); ++i)
  {
    cloud.points[i].x = p1.points[i].x;
    cloud.points[i].y = p1.points[i].y;
    cloud.points[i].z = p1.points[i].z;
    if(std::isnan(p1.points[i].x) ||
       std::isnan(p1.points[i].y) ||
       std::isnan(p1.points[i].z))
    {
      cloud.points[i].r = std::numeric_limits<int>::quiet_NaN();
      cloud.points[i].c = std::numeric_limits<int>::quiet_NaN();
    }
    else
    {
      cloud.points[i].r = p2.points[i].r;
      cloud.points[i].c = p2.points[i].c;
    }
  }
}

void PointXYZRC2PointXYZ(pcl::PointCloud<PointXYZRC> &temp, pcl::PointCloud<pcl::PointXYZ> &cloud)
{
  cloud.points.clear();
  cloud.width  = temp.width;
  cloud.height = temp.height;
  cloud.is_dense = false;
  cloud.points.resize (cloud.width * cloud.height);

  for(size_t i = 0; i < cloud.points.size(); ++i)
  {
    cloud.points[i].x = temp.points[i].x;
    cloud.points[i].y = temp.points[i].y;
    cloud.points[i].z = temp.points[i].z;
  }
}

#endif //PCLFUNCTION_HPP
