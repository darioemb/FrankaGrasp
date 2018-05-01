; Auto-generated. Do not edit!


(cl:in-package agile_grasp-msg)


;//! \htmlinclude Grasps.msg.html

(cl:defclass <Grasps> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (grasps
    :reader grasps
    :initarg :grasps
    :type (cl:vector agile_grasp-msg:Grasp)
   :initform (cl:make-array 0 :element-type 'agile_grasp-msg:Grasp :initial-element (cl:make-instance 'agile_grasp-msg:Grasp))))
)

(cl:defclass Grasps (<Grasps>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Grasps>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Grasps)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name agile_grasp-msg:<Grasps> is deprecated: use agile_grasp-msg:Grasps instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Grasps>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader agile_grasp-msg:header-val is deprecated.  Use agile_grasp-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'grasps-val :lambda-list '(m))
(cl:defmethod grasps-val ((m <Grasps>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader agile_grasp-msg:grasps-val is deprecated.  Use agile_grasp-msg:grasps instead.")
  (grasps m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Grasps>) ostream)
  "Serializes a message object of type '<Grasps>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'grasps))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'grasps))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Grasps>) istream)
  "Deserializes a message object of type '<Grasps>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
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
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Grasps>)))
  "Returns string type for a message object of type '<Grasps>"
  "agile_grasp/Grasps")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Grasps)))
  "Returns string type for a message object of type 'Grasps"
  "agile_grasp/Grasps")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Grasps>)))
  "Returns md5sum for a message object of type '<Grasps>"
  "3b718d1a7961f2593896b8cec7c8808e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Grasps)))
  "Returns md5sum for a message object of type 'Grasps"
  "3b718d1a7961f2593896b8cec7c8808e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Grasps>)))
  "Returns full string definition for message of type '<Grasps>"
  (cl:format cl:nil "Header header~%agile_grasp/Grasp[] grasps~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: agile_grasp/Grasp~%geometry_msgs/Vector3 center~%geometry_msgs/Vector3 axis~%geometry_msgs/Vector3 approach~%geometry_msgs/Vector3 surface_center~%std_msgs/Float32 width~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: std_msgs/Float32~%float32 data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Grasps)))
  "Returns full string definition for message of type 'Grasps"
  (cl:format cl:nil "Header header~%agile_grasp/Grasp[] grasps~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: agile_grasp/Grasp~%geometry_msgs/Vector3 center~%geometry_msgs/Vector3 axis~%geometry_msgs/Vector3 approach~%geometry_msgs/Vector3 surface_center~%std_msgs/Float32 width~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: std_msgs/Float32~%float32 data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Grasps>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'grasps) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Grasps>))
  "Converts a ROS message object to a list"
  (cl:list 'Grasps
    (cl:cons ':header (header msg))
    (cl:cons ':grasps (grasps msg))
))
