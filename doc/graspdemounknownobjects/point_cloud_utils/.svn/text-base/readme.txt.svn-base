# to start the point_cloud_util util node:
rosrun point_cloud_utils util
# To start gui:
rosrun dynamic_reconfigure reconfigure_gui 
# To start 3d visualizer
# Green points are the cleaned point cloud, blue points are the removed outliers, red points are the removed plane.
rosrun point_cloud_utils 3d_viewer /point_cloud_utils/cleaned_point_cloud /point_cloud_utils/outliers /point_cloud_utils/plane
# To start adept_server and armRequest
rosrun adept_server adept_server.py
rosrun arm_geometry tf_mat_srv.py


# Description of the various options in the gui
run:  Turn the node on/off
debug:  For internal use only.
remove_plane:  Uses RANSAC (Random Sample Consensus) to remove the largest plane in the point cloud.  Turns this on/off.
ransac_tol:  Specifies the distance from the plane that we still consider as part of the plane.
ransac_iter:  Number of iterations to run the RANSAC for.  With more iterations, you get a better statistical guarantee that you can remove the largest plane in the point cloud, at the cost of more computation.
remove_outliers:  Uses statistical outlier removal to outliers, but this does not work very well.
outlier_mean_k:  The number of points (k) to use for mean distance estimation.
outlier_StddevMulThresh:  The standard deviation multiplier threshold. All points outside the +/- * std_mul will be considered outliers.
remove_user_plane:  Whether or not to remove a user-specified plane.  All points with ax+by+cz+d >= 0 are kept.
remove_user_plane_a - remove_user_plane_d:  User-specified parameters of the plane.
write_binary:  Whether to write the file in binary or ascii mode.
write_ascii_precision:  If writing in ascii, specifies the number of digits of precision to write.
write_next:  If on, the node will output the next point cloud that it receives, then it will turn this option off.  To write, turn this option on, and WAIT UNTIL IT IS TURNED OFF before changing any other settings.
write_file_prefix, write_file_num:  File will be saved as (PREFIX)(NUM).txt

#################################################
# Advanced commands, not recommended for use

# estimate normals
roslaunch point_cloud_utils estimate_normal.launch
# save to pcd
rosrun pcl_ros pointcloud_to_pcd input:=/point_cloud_utils/transformed_normals
# visualize
rosrun pcl_visualization pcd_viewer -normals 100 ./1288967588.179153919.pcd
# visualize
rosrun point_cloud_utils 3d_viewer -scale 0.025 -n 3 /point_cloud_utils/transformed_normals
rosrun point_cloud_utils 3d_viewer -scale 0.025 -n 3 /point_cloud_utils/normals

