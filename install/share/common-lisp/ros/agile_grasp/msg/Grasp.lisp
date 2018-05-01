; Auto-generated. Do not edit!


(cl:in-package agile_grasp-msg)


;//! \htmlinclude Grasp.msg.html

(cl:defclass <Grasp> (roslisp-msg-protocol:ros-message)
  ((center
    :reader center
    :initarg :center
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (axis
    :reader axis
    :initarg :axis
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (approach
    :reader approach
    :initarg :approach
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (surface_center
    :reader surface_center
    :initarg :surface_center
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (width
    :reader width
    :initarg :width
    :type std_msgs-msg:Float32
    :initform (cl:make-instance 'std_msgs-msg:Float32)))
)

(cl:defclass Grasp (<Grasp>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Grasp>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Grasp)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name agile_grasp-msg:<Grasp> is deprecated: use agile_grasp-msg:Grasp instead.")))

(cl:ensure-generic-function 'center-val :lambda-list '(m))
(cl:defmethod center-val ((m <Grasp>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader agile_grasp-msg:center-val is deprecated.  Use agile_grasp-msg:center instead.")
  (center m))

(cl:ensure-generic-function 'axis-val :lambda-list '(m))
(cl:defmethod axis-val ((m <Grasp>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader agile_grasp-msg:axis-val is deprecated.  Use agile_grasp-msg:axis instead.")
  (axis m))

(cl:ensure-generic-function 'approach-val :lambda-list '(m))
(cl:defmethod approach-val ((m <Grasp>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader agile_grasp-msg:approach-val is deprecated.  Use agile_grasp-msg:approach instead.")
  (approach m))

(cl:ensure-generic-function 'surface_center-val :lambda-list '(m))
(cl:defmethod surface_center-val ((m <Grasp>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader agile_grasp-msg:surface_center-val is deprecated.  Use agile_grasp-msg:surface_center instead.")
  (surface_center m))

(cl:ensure-generic-function 'width-val :lambda-list '(m))
(cl:defmethod width-val ((m <Grasp>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader agile_grasp-msg:width-val is deprecated.  Use agile_grasp-msg:width instead.")
  (width m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Grasp>) ostream)
  "Serializes a message object of type '<Grasp>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'center) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'axis) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'approach) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'surface_center) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'width) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Grasp>) istream)
  "Deserializes a message object of type '<Grasp>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'center) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'axis) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'approach) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'surface_center) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'width) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Grasp>)))
  "Returns string type for a message object of type '<Grasp>"
  "agile_grasp/Grasp")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Grasp)))
  "Returns string type for a message object of type 'Grasp"
  "agile_grasp/Grasp")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Grasp>)))
  "Returns md5sum for a message object of type '<Grasp>"
  "e812ccd1fa0a0ad5be105b582346ad98")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Grasp)))
  "Returns md5sum for a message object of type 'Grasp"
  "e812ccd1fa0a0ad5be105b582346ad98")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Grasp>)))
  "Returns full string definition for message of type '<Grasp>"
  (cl:format cl:nil "geometry_msgs/Vector3 center~%geometry_msgs/Vector3 axis~%geometry_msgs/Vector3 approach~%geometry_msgs/Vector3 surface_center~%std_msgs/Float32 width~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: std_msgs/Float32~%float32 data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Grasp)))
  "Returns full string definition for message of type 'Grasp"
  (cl:format cl:nil "geometry_msgs/Vector3 center~%geometry_msgs/Vector3 axis~%geometry_msgs/Vector3 approach~%geometry_msgs/Vector3 surface_center~%std_msgs/Float32 width~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: std_msgs/Float32~%float32 data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Grasp>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'center))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'axis))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'approach))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'surface_center))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'width))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Grasp>))
  "Converts a ROS message object to a list"
  (cl:list 'Grasp
    (cl:cons ':center (center msg))
    (cl:cons ':axis (axis msg))
    (cl:cons ':approach (approach msg))
    (cl:cons ':surface_center (surface_center msg))
    (cl:cons ':width (width msg))
))
