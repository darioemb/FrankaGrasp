;; Auto-generated. Do not edit!


(when (boundp 'agile_grasp::FindGrasps)
  (if (not (find-package "AGILE_GRASP"))
    (make-package "AGILE_GRASP"))
  (shadow 'FindGrasps (find-package "AGILE_GRASP")))
(unless (find-package "AGILE_GRASP::FINDGRASPS")
  (make-package "AGILE_GRASP::FINDGRASPS"))
(unless (find-package "AGILE_GRASP::FINDGRASPSREQUEST")
  (make-package "AGILE_GRASP::FINDGRASPSREQUEST"))
(unless (find-package "AGILE_GRASP::FINDGRASPSRESPONSE")
  (make-package "AGILE_GRASP::FINDGRASPSRESPONSE"))

(in-package "ROS")

(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))


(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass agile_grasp::FindGraspsRequest
  :super ros::object
  :slots (_object_cloud _approach_offset ))

(defmethod agile_grasp::FindGraspsRequest
  (:init
   (&key
    ((:object_cloud __object_cloud) (instance sensor_msgs::PointCloud2 :init))
    ((:approach_offset __approach_offset) 0.0)
    )
   (send-super :init)
   (setq _object_cloud __object_cloud)
   (setq _approach_offset (float __approach_offset))
   self)
  (:object_cloud
   (&rest __object_cloud)
   (if (keywordp (car __object_cloud))
       (send* _object_cloud __object_cloud)
     (progn
       (if __object_cloud (setq _object_cloud (car __object_cloud)))
       _object_cloud)))
  (:approach_offset
   (&optional __approach_offset)
   (if __approach_offset (setq _approach_offset __approach_offset)) _approach_offset)
  (:serialization-length
   ()
   (+
    ;; sensor_msgs/PointCloud2 _object_cloud
    (send _object_cloud :serialization-length)
    ;; float64 _approach_offset
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; sensor_msgs/PointCloud2 _object_cloud
       (send _object_cloud :serialize s)
     ;; float64 _approach_offset
       (sys::poke _approach_offset (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; sensor_msgs/PointCloud2 _object_cloud
     (send _object_cloud :deserialize buf ptr-) (incf ptr- (send _object_cloud :serialization-length))
   ;; float64 _approach_offset
     (setq _approach_offset (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass agile_grasp::FindGraspsResponse
  :super ros::object
  :slots (_grasps _grasp_poses _approach_poses ))

(defmethod agile_grasp::FindGraspsResponse
  (:init
   (&key
    ((:grasps __grasps) (let (r) (dotimes (i 0) (push (instance agile_grasp::Grasp :init) r)) r))
    ((:grasp_poses __grasp_poses) (let (r) (dotimes (i 0) (push (instance geometry_msgs::PoseStamped :init) r)) r))
    ((:approach_poses __approach_poses) (let (r) (dotimes (i 0) (push (instance geometry_msgs::PoseStamped :init) r)) r))
    )
   (send-super :init)
   (setq _grasps __grasps)
   (setq _grasp_poses __grasp_poses)
   (setq _approach_poses __approach_poses)
   self)
  (:grasps
   (&rest __grasps)
   (if (keywordp (car __grasps))
       (send* _grasps __grasps)
     (progn
       (if __grasps (setq _grasps (car __grasps)))
       _grasps)))
  (:grasp_poses
   (&rest __grasp_poses)
   (if (keywordp (car __grasp_poses))
       (send* _grasp_poses __grasp_poses)
     (progn
       (if __grasp_poses (setq _grasp_poses (car __grasp_poses)))
       _grasp_poses)))
  (:approach_poses
   (&rest __approach_poses)
   (if (keywordp (car __approach_poses))
       (send* _approach_poses __approach_poses)
     (progn
       (if __approach_poses (setq _approach_poses (car __approach_poses)))
       _approach_poses)))
  (:serialization-length
   ()
   (+
    ;; agile_grasp/Grasp[] _grasps
    (apply #'+ (send-all _grasps :serialization-length)) 4
    ;; geometry_msgs/PoseStamped[] _grasp_poses
    (apply #'+ (send-all _grasp_poses :serialization-length)) 4
    ;; geometry_msgs/PoseStamped[] _approach_poses
    (apply #'+ (send-all _approach_poses :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; agile_grasp/Grasp[] _grasps
     (write-long (length _grasps) s)
     (dolist (elem _grasps)
       (send elem :serialize s)
       )
     ;; geometry_msgs/PoseStamped[] _grasp_poses
     (write-long (length _grasp_poses) s)
     (dolist (elem _grasp_poses)
       (send elem :serialize s)
       )
     ;; geometry_msgs/PoseStamped[] _approach_poses
     (write-long (length _approach_poses) s)
     (dolist (elem _approach_poses)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; agile_grasp/Grasp[] _grasps
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _grasps (let (r) (dotimes (i n) (push (instance agile_grasp::Grasp :init) r)) r))
     (dolist (elem- _grasps)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; geometry_msgs/PoseStamped[] _grasp_poses
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _grasp_poses (let (r) (dotimes (i n) (push (instance geometry_msgs::PoseStamped :init) r)) r))
     (dolist (elem- _grasp_poses)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; geometry_msgs/PoseStamped[] _approach_poses
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _approach_poses (let (r) (dotimes (i n) (push (instance geometry_msgs::PoseStamped :init) r)) r))
     (dolist (elem- _approach_poses)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(defclass agile_grasp::FindGrasps
  :super ros::object
  :slots ())

(setf (get agile_grasp::FindGrasps :md5sum-) "3e976888ec7075e7e22c71cd9a0d3700")
(setf (get agile_grasp::FindGrasps :datatype-) "agile_grasp/FindGrasps")
(setf (get agile_grasp::FindGrasps :request) agile_grasp::FindGraspsRequest)
(setf (get agile_grasp::FindGrasps :response) agile_grasp::FindGraspsResponse)

(defmethod agile_grasp::FindGraspsRequest
  (:response () (instance agile_grasp::FindGraspsResponse :init)))

(setf (get agile_grasp::FindGraspsRequest :md5sum-) "3e976888ec7075e7e22c71cd9a0d3700")
(setf (get agile_grasp::FindGraspsRequest :datatype-) "agile_grasp/FindGraspsRequest")
(setf (get agile_grasp::FindGraspsRequest :definition-)
      "sensor_msgs/PointCloud2 object_cloud
float64 approach_offset

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
---
agile_grasp/Grasp[] grasps
geometry_msgs/PoseStamped[] grasp_poses
geometry_msgs/PoseStamped[] approach_poses



================================================================================
MSG: agile_grasp/Grasp
geometry_msgs/Vector3 center
geometry_msgs/Vector3 axis
geometry_msgs/Vector3 approach
geometry_msgs/Vector3 surface_center
std_msgs/Float32 width

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: std_msgs/Float32
float32 data
================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

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
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
")

(setf (get agile_grasp::FindGraspsResponse :md5sum-) "3e976888ec7075e7e22c71cd9a0d3700")
(setf (get agile_grasp::FindGraspsResponse :datatype-) "agile_grasp/FindGraspsResponse")
(setf (get agile_grasp::FindGraspsResponse :definition-)
      "sensor_msgs/PointCloud2 object_cloud
float64 approach_offset

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
---
agile_grasp/Grasp[] grasps
geometry_msgs/PoseStamped[] grasp_poses
geometry_msgs/PoseStamped[] approach_poses



================================================================================
MSG: agile_grasp/Grasp
geometry_msgs/Vector3 center
geometry_msgs/Vector3 axis
geometry_msgs/Vector3 approach
geometry_msgs/Vector3 surface_center
std_msgs/Float32 width

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: std_msgs/Float32
float32 data
================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

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
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
")



(provide :agile_grasp/FindGrasps "3e976888ec7075e7e22c71cd9a0d3700")


