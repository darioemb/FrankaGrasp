; Auto-generated. Do not edit!


(cl:in-package agile_grasp-srv)


;//! \htmlinclude FindGrasps-request.msg.html

(cl:defclass <FindGrasps-request> (roslisp-msg-protocol:ros-message)
  ((object_cloud
    :reader object_cloud
    :initarg :object_cloud
    :type sensor_msgs-msg:PointCloud2
    :initform (cl:make-instance 'sensor_msgs-msg:PointCloud2))
   (approach_offset
    :reader approach_offset
    :initarg :approach_offset
    :type cl:float
    :initform 0.0))
)

(cl:defclass FindGrasps-request (<FindGrasps-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FindGrasps-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FindGrasps-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name agile_grasp-srv:<FindGrasps-request> is deprecated: use agile_grasp-srv:FindGrasps-request instead.")))

(cl:ensure-generic-function 'object_cloud-val :lambda-list '(m))
(cl:defmethod object_cloud-val ((m <FindGrasps-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader agile_grasp-srv:object_cloud-val is deprecated.  Use agile_grasp-srv:object_cloud instead.")
  (object_cloud m))

(cl:ensure-generic-function 'approach_offset-val :lambda-list '(m))
(cl:defmethod approach_offset-val ((m <FindGrasps-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader agile_grasp-srv:approach_offset-val is deprecated.  Use agile_grasp-srv:approach_offset instead.")
  (approach_offset m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FindGrasps-request>) ostream)
  "Serializes a message object of type '<FindGrasps-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'object_cloud) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'approach_offset))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FindGrasps-request>) istream)
  "Deserializes a message object of type '<FindGrasps-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'object_cloud) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'approach_offset) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FindGrasps-request>)))
  "Returns string type for a service object of type '<FindGrasps-request>"
  "agile_grasp/FindGraspsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FindGrasps-request)))
  "Returns string type for a service object of type 'FindGrasps-request"
  "agile_grasp/FindGraspsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FindGrasps-request>)))
  "Returns md5sum for a message object of type '<FindGrasps-request>"
  "3e976888ec7075e7e22c71cd9a0d3700")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FindGrasps-request)))
  "Returns md5sum for a message object of type 'FindGrasps-request"
  "3e976888ec7075e7e22c71cd9a0d3700")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FindGrasps-request>)))
  "Returns full string definition for message of type '<FindGrasps-request>"
  (cl:format cl:nil "sensor_msgs/PointCloud2 object_cloud~%float64 approach_offset~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FindGrasps-request)))
  "Returns full string definition for message of type 'FindGrasps-request"
  (cl:format cl:nil "sensor_msgs/PointCloud2 object_cloud~%float64 approach_offset~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FindGrasps-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'object_cloud))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FindGrasps-request>))
  "Converts a ROS message object to a list"
  (cl:list 'FindGrasps-request
    (cl:cons ':object_cloud (object_cloud msg))
    (cl:cons ':approach_offset (approach_offset msg))
))
;//! \htmlinclude FindGrasps-response.msg.html

(cl:defclass <FindGrasps-response> (roslisp-msg-protocol:ros-message)
  ((grasps
    :reader grasps
    :initarg :grasps
    :type (cl:vector agile_grasp-msg:Grasp)
   :initform (cl:make-array 0 :element-type 'agile_grasp-msg:Grasp :initial-element (cl:make-instance 'agile_grasp-msg:Grasp)))
   (grasp_poses
    :reader grasp_poses
    :initarg :grasp_poses
    :type (cl:vector geometry_msgs-msg:PoseStamped)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:PoseStamped :initial-element (cl:make-instance 'geometry_msgs-msg:PoseStamped)))
   (approach_poses
    :reader approach_poses
    :initarg :approach_poses
    :type (cl:vector geometry_msgs-msg:PoseStamped)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:PoseStamped :initial-element (cl:make-instance 'geometry_msgs-msg:PoseStamped))))
)

(cl:defclass FindGrasps-response (<FindGrasps-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FindGrasps-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FindGrasps-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name agile_grasp-srv:<FindGrasps-response> is deprecated: use agile_grasp-srv:FindGrasps-response instead.")))

(cl:ensure-generic-function 'grasps-val :lambda-list '(m))
(cl:defmethod grasps-val ((m <FindGrasps-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader agile_grasp-srv:grasps-val is deprecated.  Use agile_grasp-srv:grasps instead.")
  (grasps m))

(cl:ensure-generic-function 'grasp_poses-val :lambda-list '(m))
(cl:defmethod grasp_poses-val ((m <FindGrasps-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader agile_grasp-srv:grasp_poses-val is deprecated.  Use agile_grasp-srv:grasp_poses instead.")
  (grasp_poses m))

(cl:ensure-generic-function 'approach_poses-val :lambda-list '(m))
(cl:defmethod approach_poses-val ((m <FindGrasps-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader agile_grasp-srv:approach_poses-val is deprecated.  Use agile_grasp-srv:approach_poses instead.")
  (approach_poses m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FindGrasps-response>) ostream)
  "Serializes a message object of type '<FindGrasps-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'grasps))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'grasps))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'grasp_poses))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'grasp_poses))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'approach_poses))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'approach_poses))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FindGrasps-response>) istream)
  "Deserializes a message object of type '<FindGrasps-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'grasps) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'grasps)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'agile_grasp-msg:Grasp))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'grasp_poses) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'grasp_poses)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:PoseStamped))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'approach_poses) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'approach_poses)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:PoseStamped))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FindGrasps-response>)))
  "Returns string type for a service object of type '<FindGrasps-response>"
  "agile_grasp/FindGraspsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FindGrasps-response)))
  "Returns string type for a service object of type 'FindGrasps-response"
  "agile_grasp/FindGraspsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FindGrasps-response>)))
  "Returns md5sum for a message object of type '<FindGrasps-response>"
  "3e976888ec7075e7e22c71cd9a0d3700")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FindGrasps-response)))
  "Returns md5sum for a message object of type 'FindGrasps-response"
  "3e976888ec7075e7e22c71cd9a0d3700")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FindGrasps-response>)))
  "Returns full string definition for message of type '<FindGrasps-response>"
  (cl:format cl:nil "agile_grasp/Grasp[] grasps~%geometry_msgs/PoseStamped[] grasp_poses~%geometry_msgs/PoseStamped[] approach_poses~%~%~%~%================================================================================~%MSG: agile_grasp/Grasp~%geometry_msgs/Vector3 center~%geometry_msgs/Vector3 axis~%geometry_msgs/Vector3 approach~%geometry_msgs/Vector3 surface_center~%std_msgs/Float32 width~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: std_msgs/Float32~%float32 data~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FindGrasps-response)))
  "Returns full string definition for message of type 'FindGrasps-response"
  (cl:format cl:nil "agile_grasp/Grasp[] grasps~%geometry_msgs/PoseStamped[] grasp_poses~%geometry_msgs/PoseStamped[] approach_poses~%~%~%~%================================================================================~%MSG: agile_grasp/Grasp~%geometry_msgs/Vector3 center~%geometry_msgs/Vector3 axis~%geometry_msgs/Vector3 approach~%geometry_msgs/Vector3 surface_center~%std_msgs/Float32 width~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: std_msgs/Float32~%float32 data~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FindGrasps-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'grasps) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'grasp_poses) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'approach_poses) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FindGrasps-response>))
  "Converts a ROS message object to a list"
  (cl:list 'FindGrasps-response
    (cl:cons ':grasps (grasps msg))
    (cl:cons ':grasp_poses (grasp_poses msg))
    (cl:cons ':approach_poses (approach_poses msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'FindGrasps)))
  'FindGrasps-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'FindGrasps)))
  'FindGrasps-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FindGrasps)))
  "Returns string type for a service object of type '<FindGrasps>"
  "agile_grasp/FindGrasps")