;; Auto-generated. Do not edit!


(when (boundp 'agile_grasp::CloudSized)
  (if (not (find-package "AGILE_GRASP"))
    (make-package "AGILE_GRASP"))
  (shadow 'CloudSized (find-package "AGILE_GRASP")))
(unless (find-package "AGILE_GRASP::CLOUDSIZED")
  (make-package "AGILE_GRASP::CLOUDSIZED"))

(in-package "ROS")
;;//! \htmlinclude CloudSized.msg.html
(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass agile_grasp::CloudSized
  :super ros::object
  :slots (_cloud _size_left ))

(defmethod agile_grasp::CloudSized
  (:init
   (&key
    ((:cloud __cloud) (instance sensor_msgs::PointCloud2 :init))
    ((:size_left __size_left) (instance std_msgs::Int64 :init))
    )
   (send-super :init)
   (setq _cloud __cloud)
   (setq _size_left __size_left)
   self)
  (:cloud
   (&rest __cloud)
   (if (keywordp (car __cloud))
       (send* _cloud __cloud)
     (progn
       (if __cloud (setq _cloud (car __cloud)))
       _cloud)))
  (:size_left
   (&rest __size_left)
   (if (keywordp (car __size_left))
       (send* _size_left __size_left)
     (progn
       (if __size_left (setq _size_left (car __size_left)))
       _size_left)))
  (:serialization-length
   ()
   (+
    ;; sensor_msgs/PointCloud2 _cloud
    (send _cloud :serialization-length)
    ;; std_msgs/Int64 _size_left
    (send _size_left :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; sensor_msgs/PointCloud2 _cloud
       (send _cloud :serialize s)
     ;; std_msgs/Int64 _size_left
       (send _size_left :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; sensor_msgs/PointCloud2 _cloud
     (send _cloud :deserialize buf ptr-) (incf ptr- (send _cloud :serialization-length))
   ;; std_msgs/Int64 _size_left
     (send _size_left :deserialize buf ptr-) (incf ptr- (send _size_left :serialization-length))
   ;;
   self)
  )

(setf (get agile_grasp::CloudSized :md5sum-) "4250ffa867aa297b69ecf90acee4dc17")
(setf (get agile_grasp::CloudSized :datatype-) "agile_grasp/CloudSized")
(setf (get agile_grasp::CloudSized :definition-)
      "sensor_msgs/PointCloud2 cloud
std_msgs/Int64 size_left

================================================================================
MSG: sensor_msgs/PointCloud2
# This message holds a collection of N-dimensional points, which may
# contain additional information such as normals, intensity, etc. The
# point data is stored as a binary blob, its layout described by the
# contents of the \"fields\" array.

# The point cloud data may be organized 2d (image-like) or 1d
# (unordered). Point clouds organized as 2d images may be produced by
# camera depth sensors such as stereo or time-of-flight.

# Time of sensor data acquisition, and the coordinate frame ID (for 3d
# points).
Header header

# 2D structure of the point cloud. If the cloud is unordered, height is
# 1 and width is the length of the point cloud.
uint32 height
uint32 width

# Describes the channels and their layout in the binary data blob.
PointField[] fields

bool    is_bigendian # Is this data bigendian?
uint32  point_step   # Length of a point in bytes
uint32  row_step     # Length of a row in bytes
uint8[] data         # Actual point data, size is (row_step*height)

bool is_dense        # True if there are no invalid points

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: sensor_msgs/PointField
# This message holds the description of one point entry in the
# PointCloud2 message format.
uint8 INT8    = 1
uint8 UINT8   = 2
uint8 INT16   = 3
uint8 UINT16  = 4
uint8 INT32   = 5
uint8 UINT32  = 6
uint8 FLOAT32 = 7
uint8 FLOAT64 = 8

string name      # Name of field
uint32 offset    # Offset from start of point struct
uint8  datatype  # Datatype enumeration, see above
uint32 count     # How many elements in the field

================================================================================
MSG: std_msgs/Int64
int64 data
")



(provide :agile_grasp/CloudSized "4250ffa867aa297b69ecf90acee4dc17")


