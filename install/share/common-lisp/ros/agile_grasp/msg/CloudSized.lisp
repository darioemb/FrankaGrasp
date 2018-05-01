; Auto-generated. Do not edit!


(cl:in-package agile_grasp-msg)


;//! \htmlinclude CloudSized.msg.html

(cl:defclass <CloudSized> (roslisp-msg-protocol:ros-message)
  ((cloud
    :reader cloud
    :initarg :cloud
    :type sensor_msgs-msg:PointCloud2
    :initform (cl:make-instance 'sensor_msgs-msg:PointCloud2))
   (size_left
    :reader size_left
    :initarg :size_left
    :type std_msgs-msg:Int64
    :initform (cl:make-instance 'std_msgs-msg:Int64)))
)

(cl:defclass CloudSized (<CloudSized>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CloudSized>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CloudSized)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name agile_grasp-msg:<CloudSized> is deprecated: use agile_grasp-msg:CloudSized instead.")))

(cl:ensure-generic-function 'cloud-val :lambda-list '(m))
(cl:defmethod cloud-val ((m <CloudSized>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader agile_grasp-msg:cloud-val is deprecated.  Use agile_grasp-msg:cloud instead.")
  (cloud m))

(cl:ensure-generic-function 'size_left-val :lambda-list '(m))
(cl:defmethod size_left-val ((m <CloudSized>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader agile_grasp-msg:size_left-val is deprecated.  Use agile_grasp-msg:size_left instead.")
  (size_left m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CloudSized>) ostream)
  "Serializes a message object of type '<CloudSized>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'cloud) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'size_left) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CloudSized>) istream)
  "Deserializes a message object of type '<CloudSized>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'cloud) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'size_left) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CloudSized>)))
  "Returns string type for a message object of type '<CloudSized>"
  "agile_grasp/CloudSized")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CloudSized)))
  "Returns string type for a message object of type 'CloudSized"
  "agile_grasp/CloudSized")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CloudSized>)))
  "Returns md5sum for a message object of type '<CloudSized>"
  "4250ffa867aa297b69ecf90acee4dc17")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CloudSized)))
  "Returns md5sum for a message object of type 'CloudSized"
  "4250ffa867aa297b69ecf90acee4dc17")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CloudSized>)))
  "Returns full string definition for message of type '<CloudSized>"
  (cl:format cl:nil "sensor_msgs/PointCloud2 cloud~%std_msgs/Int64 size_left~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%================================================================================~%MSG: std_msgs/Int64~%int64 data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CloudSized)))
  "Returns full string definition for message of type 'CloudSized"
  (cl:format cl:nil "sensor_msgs/PointCloud2 cloud~%std_msgs/Int64 size_left~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%================================================================================~%MSG: std_msgs/Int64~%int64 data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CloudSized>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'cloud))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'size_left))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CloudSized>))
  "Converts a ROS message object to a list"
  (cl:list 'CloudSized
    (cl:cons ':cloud (cloud msg))
    (cl:cons ':size_left (size_left msg))
))
