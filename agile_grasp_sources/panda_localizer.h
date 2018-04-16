#pragma once
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <agile_grasp/localization.h>
#include <eigen_conversions/eigen_msg.h>
#include <agile_grasp/Grasp.h>
#include <agile_grasp/Grasps.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class PandaLocalizer
{
public:
	struct Parameters
	{
		int num_threads;

		double finger_width;
		double hand_outer_diameter;
		double hand_depth;
		double hand_height;
		double init_bite;

		int plotting_mode;
	};

	PandaLocalizer(const std::string& svm_filename, const Parameters& params);
	~PandaLocalizer(){};

	void localizeGrasps();
	void subscribe(const std::string& cloud_topic, ros::NodeHandle& nh);
private:
	sensor_msgs::PointCloud2::Ptr pointCloud2;
	PointCloud::Ptr pointCloud;
	std::vector<GraspHypothesis> antipodal_hands; ///< the antipodal grasps predicted by the svm_filename
	std::vector<GraspHypothesis> hands; ///< the grasp hypotheses found by the hand search
	Localization* localization; ///< a pointer to a localization object
	ros::Subscriber cloud_topic;

	void cloud2_callback(const sensor_msgs::PointCloud2ConstPtr& msg);
};