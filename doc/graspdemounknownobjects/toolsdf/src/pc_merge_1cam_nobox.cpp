/*
 *
 * David Fischinger
 * Vienna University of Technology
 * 26.03.2012
 *
 * input:
 *
 *   pointcloud from camera1 read from topic /SS/camera/depth/points
 *   pointcloud from camera2 read from topic /SS/camera2/depth/points
 *
 * output:
 *   pointcloud after change of coordinate system (modul is also capable of merging points from 2 cameras, filtering and cutting points)
 *
 *   output point cloud w.r.t. tf_frame /table_center_frame on topic /SS/points2_without_basket_wcs
 *
 *
 * PARAMETERS:
 *
 * 	 use_two_cams = false;			//if one ore two cameras are used
 *
 */


#include <pc_merge_with_basket_rot.hpp>

CPCMerge::CPCMerge(ros::NodeHandle nh_)
{
  nh = nh_;
  pc_cam1_filled = pc_cam2_filled = false;
  use_two_cams = false;
  //define publishers
  pc_for_basketdet_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/SS/points2_without_basket_wcs",1);
  position_basket_pub = nh.advertise<std_msgs::String>("/SS/basket_position",1);
  nr_segmented_pc_pub = nh.advertise<std_msgs::String>("pc_merge/nr_segmented_pcs", 1); //publish nr of segmented point clouds

  //define subscriber
  pc_cam1_sub = nh.subscribe("/SS/camera/depth/points",1, &CPCMerge::pc_cam1_callback, this);
  if (use_two_cams)
  {
	pc_cam2_sub = nh.subscribe("/SS/camera2/depth/points",1, &CPCMerge::pc_cam2_callback, this);
  }
}

void CPCMerge::pc_cam1_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pcl_in)
{
  m.lock();

  ROS_INFO("pc_cam1 received");

  this->pc_cam1_filled = false;

  //search for tf transform for pc from cam1
  bool foundTransform = tf_listener.waitForTransform("/table_center_frame", "/openni_rgb_optical_frame",
		                                             (*pcl_in).header.stamp, ros::Duration(3.0));
  if (!foundTransform)
  {
	ROS_WARN("No pc_cam1 transform found");
	m.unlock();
	return;
  }

  ROS_INFO("Transform pc_cam1: table_center_frame to world_or_frame found");
  pcl_ros::transformPointCloud("/table_center_frame", *pcl_in, pc_cam1, tf_listener);
  this->pc_cam1_filled = true;

  //publishes point cloud if pc from cam2 already arrived or if only one camera is used
  if ( this->pc_cam2_filled or !this->use_two_cams)
  {
	publish_merged_pc();
  }

  m.unlock();
}


void CPCMerge::pc_cam2_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pcl_in)
{
  m.lock();

  ROS_INFO("pc_cam2 received");

  this->pc_cam2_filled = false;

  //search for tf transform for pc from cam1
  bool foundTransform = tf_listener.waitForTransform("/table_center_frame", "/openni_depth_optical_frame2",
		                                              (*pcl_in).header.stamp, ros::Duration(3.0));
  if (!foundTransform)
  {
	ROS_WARN("No pc_cam2 transform found");
	m.unlock();
	return;
  }

  ROS_INFO("Transform pc_cam2: openni_depth_optical_frame to world found");
  pcl_ros::transformPointCloud("/table_center_frame", *pcl_in, pc_cam2, tf_listener);

  this->pc_cam2_filled = true;

  if(this->pc_cam1_filled)
  {
    publish_merged_pc();
  }

  m.unlock();

}

void CPCMerge::publish_merged_pc()
{
	ROS_INFO("publish_merged_pc() started");
	pcl::PointCloud<pcl::PointXYZ> pcl_cloud_merged;
	pcl_cloud_merged = pc_cam1;

	// if second camera is used, merge both point clouds
	// (otherwise only pc from one camera is used)
	if(this->use_two_cams)
	{
	  pcl_cloud_merged += pc_cam2;
	}

	//Filter Data (with basket detection and basket point elimination)
	filter_pc(pcl_cloud_merged, false); //18.10.2011 last entry false <=> coordinates of highest points of scene are published instead of basket center point

    //publish manipulated and merged point cloud data (in coordinate system of table_center_frame)
    pc_for_basketdet_out = pcl_cloud_merged;
    pc_for_basketdet_out.header.frame_id = "/table_center_frame";
	pc_for_basketdet_pub.publish(pc_for_basketdet_out);

	ROS_INFO("basket detection published");

	//PUBLISH POINTCLOUD WITHOUT SEGMENTATION and fixed (=1) this->nr_segmented_pc_pub.publish(j_string);
	std_msgs::String tmp_string;
	std::stringstream tmpss;
	tmpss << 1;
	tmp_string.data = tmpss.str();
	//next 2 lines (un)comment always together!
	nr_segmented_pc_pub.publish(tmp_string);


	//search for tf transform for pc from table_center_frame to cam1 and make transform
	bool foundTransform = tf_listener.waitForTransform("/openni_rgb_optical_frame", "/table_center_frame",
			                                           pc_cam1.header.stamp, ros::Duration(3.0));
	if (!foundTransform)
	{
	  ROS_WARN("No pc_cam1 (Kate/Karthik) transform found");
	  this->pc_cam1_filled = false;
	  this->pc_cam2_filled = false;
	  return;
	}

    this->pc_cam1_filled = false;
	this->pc_cam2_filled = false;
}

// filter point cloud and cut off points outside a defined region
//if pub_basket_position == false than the position of the highest point is returned! otherwise the center of basket
void CPCMerge::filter_pc(pcl::PointCloud<pcl::PointXYZ>& pcl_cloud_merged, bool pub_basket_position, float x_min, float x_max, float y_min, float y_max, float z_min, float z_max )
{
	// Create the filtering object
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_z (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_y (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_x (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    *cloud = pcl_cloud_merged;
	pcl::PassThrough<pcl::PointXYZ> pass;

	ROS_INFO("Filtering outliers and cutting region");
    ROS_INFO("Number of points before cutting and filtering: %d",pcl_cloud_merged.points.size());

	//Filter w.r.t. axis z
	pass.setInputCloud (cloud);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (z_min, z_max);
	//pass.setFilterLimitsNegative (true);
	pass.filter (*cloud_filtered_z);

    ROS_INFO("Number of points after cutting z-direction: %d",(*cloud_filtered_z).points.size());

	//Filter w.r.t. axis y
	pass.setInputCloud(cloud_filtered_z);
	pass.setFilterFieldName ("y");
	pass.setFilterLimits (y_min, y_max);
	pass.filter (*cloud_filtered_y);

	ROS_INFO("Number of points after cutting y-direction: %d",(*cloud_filtered_y).points.size());

	//Filter w.r.t. axis x
	pass.setInputCloud(cloud_filtered_y);
	pass.setFilterFieldName ("x");
	pass.setFilterLimits (x_min, x_max);
	pass.filter (*cloud_filtered_x);

	ROS_INFO("Number of points after cutting x-direction: %d",(*cloud_filtered_x).points.size());

	//Create the filtering object
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_filtered_x);
	sor.setMeanK(50);
	sor.setStddevMulThresh (1.0);
	sor.filter(pcl_cloud_merged);

	if (pub_basket_position == false){ // than the position of the highest point is published!
	    ROS_INFO("Number of points after cutting and filtering: %d",pcl_cloud_merged.points.size());
		float z_max = -1000;			//max heigt val for z
		float x_for_z_max = -1000;		//coordinates for x,y at position of maximal z
		float y_for_z_max = -1000;
	    for (unsigned int i = 0; i < pcl_cloud_merged.points.size(); ++i)
		{
		  if (pcl_cloud_merged.points[i].z > z_max)
		  {
			z_max = pcl_cloud_merged.points[i].z;
			x_for_z_max = pcl_cloud_merged.points[i].x;
			y_for_z_max = pcl_cloud_merged.points[i].y;
		  }
		}
	    //Publish coordinates of heighest points ("basket center" if there is no basket)
		std_msgs::String msgStrCorners;
		std::stringstream ss;
		ss << x_for_z_max << " "  << y_for_z_max << " " << 0;
		msgStrCorners.data = ss.str();
		position_basket_pub.publish(msgStrCorners);
	} else {
		basket_detection<pcl::PointXYZ>(pcl_cloud_merged,pub_basket_position);
	}

}
/*
// downsampling: reduce the number of points of the pc
void CPCMerge::downsampling_pc(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_merged )
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
    *cloud_in = *pcl_cloud_merged;

    std::cerr << "PointCloud before downsampling: " << cloud_in->width * cloud_in->height
           << " data points (" << pcl::getFieldsList (*cloud_in) << "). \n";

    // Create the VoxelGrid object
     pcl::VoxelGrid<pcl::PointXYZ> vg;
     vg.setInputCloud (cloud_in);
     vg.setLeafSize (0.01, 0.01, 0.01);
     vg.filter (*pcl_cloud_merged);

     std::cerr << "PointCloud after downsampling: " << pcl_cloud_merged->width * pcl_cloud_merged->height
            << " data points (" << pcl::getFieldsList (*pcl_cloud_merged) << "). \n";
}
*/

/*
// segment the merged, filtered, downsampled point cloud
void CPCMerge::segment_pc(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_merged )
{
	using namespace pcl;
	using namespace std;

    PointCloud<PointXYZ>::Ptr cloud_in (new PointCloud<PointXYZ>);
    *cloud_in = *pcl_cloud_merged;

    PCDWriter writer;

    // Creating the KdTree object for the search method of the extraction
    KdTree<PointXYZ>::Ptr tree (new KdTreeFLANN<PointXYZ>);
    tree->setInputCloud (cloud_in);

    vector<PointIndices> cluster_indices;
    EuclideanClusterExtraction<PointXYZ> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (50);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud( cloud_in);
    ec.extract (cluster_indices);

    int j = 0;
    for (vector<PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        PointCloud<PointXYZ>::Ptr cloud_cluster (new PointCloud<PointXYZ>);
        for (vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
            cloud_cluster->points.push_back (cloud_in->points[*pit]);

        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->header = cloud_in->header;
        cloud_cluster->is_dense = cloud_in->is_dense;

        //cerr << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << endl;
        stringstream ss;
        ss << "cloud_cluster_" << j << ".pcd";
        writer.write<PointXYZ> (ss.str (), *cloud_cluster, false);


        //publish manipulated point clusters
        sensor_msgs::PointCloud2 cluster_out;
        toROSMsg(*cloud_cluster,cluster_out);

        cluster_out.header.frame_id = cloud_in->header.frame_id;
        cloud_cluster->is_dense = cloud_in->is_dense;


        cluster_out.header = cloud_cluster->header; //new 21.11.2011
        cluster_out.header.frame_id = cloud_in->header.frame_id;
        this->pc_merged_pub.publish(cluster_out);

        j++;
    }
    std_msgs::String j_string;
    stringstream ss;
    ss << j;
    j_string.data = ss.str();
    this->nr_segmented_pc_pub.publish(j_string);
}
*/






int main (int argc, char** argv)
{
  ROS_INFO("ROS NODE pc_merge_with_basket started");
  ros::init(argc, argv, "pc_merge");
  ros::NodeHandle nh;
  //tf_listener = new tf::TransformListener();
  CPCMerge * pc_merge = new CPCMerge(nh);

  ros::spin();
  //delete tf_listener;
  return (0);
}
