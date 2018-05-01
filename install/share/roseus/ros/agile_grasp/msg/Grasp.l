;; Auto-generated. Do not edit!


(when (boundp 'agile_grasp::Grasp)
  (if (not (find-package "AGILE_GRASP"))
    (make-package "AGILE_GRASP"))
  (shadow 'Grasp (find-package "AGILE_GRASP")))
(unless (find-package "AGILE_GRASP::GRASP")
  (make-package "AGILE_GRASP::GRASP"))

(in-package "ROS")
;;//! \htmlinclude Grasp.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass agile_grasp::Grasp
  :super ros::object
  :slots (_center _axis _approach _surface_center _width ))

(defmethod agile_grasp::Grasp
  (:init
   (&key
    ((:center __center) (instance geometry_msgs::Vector3 :init))
    ((:axis __axis) (instance geometry_msgs::Vector3 :init))
    ((:approach __approach) (instance geometry_msgs::Vector3 :init))
    ((:surface_center __surface_center) (instance geometry_msgs::Vector3 :init))
    ((:width __width) (instance std_msgs::Float32 :init))
    )
   (send-super :init)
   (setq _center __center)
   (setq _axis __axis)
   (setq _approach __approach)
   (setq _surface_center __surface_center)
   (setq _width __width)
   self)
  (:center
   (&rest __center)
   (if (keywordp (car __center))
       (send* _center __center)
     (progn
       (if __center (setq _center (car __center)))
       _center)))
  (:axis
   (&rest __axis)
   (if (keywordp (car __axis))
       (send* _axis __axis)
     (progn
       (if __axis (setq _axis (car __axis)))
       _axis)))
  (:approach
   (&rest __approach)
   (if (keywordp (car __approach))
       (send* _approach __approach)
     (progn
       (if __approach (setq _approach (car __approach)))
       _approach)))
  (:surface_center
   (&rest __surface_center)
   (if (keywordp (car __surface_center))
       (send* _surface_center __surface_center)
     (progn
       (if __surface_center (setq _surface_center (car __surface_center)))
       _surface_center)))
  (:width
   (&rest __width)
   (if (keywordp (car __width))
       (send* _width __width)
     (progn
       (if __width (setq _width (car __width)))
       _width)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Vector3 _center
    (send _center :serialization-length)
    ;; geometry_msgs/Vector3 _axis
    (send _axis :serialization-length)
    ;; geometry_msgs/Vector3 _approach
    (send _approach :serialization-length)
    ;; geometry_msgs/Vector3 _surface_center
    (send _surface_center :serialization-length)
    ;; std_msgs/Float32 _width
    (send _width :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Vector3 _center
       (send _center :serialize s)
     ;; geometry_msgs/Vector3 _axis
       (send _axis :serialize s)
     ;; geometry_msgs/Vector3 _approach
       (send _approach :serialize s)
     ;; geometry_msgs/Vector3 _surface_center
       (send _surface_center :serialize s)
     ;; std_msgs/Float32 _width
       (send _width :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Vector3 _center
     (send _center :deserialize buf ptr-) (incf ptr- (send _center :serialization-length))
   ;; geometry_msgs/Vector3 _axis
     (send _axis :deserialize buf ptr-) (incf ptr- (send _axis :serialization-length))
   ;; geometry_msgs/Vector3 _approach
     (send _approach :deserialize buf ptr-) (incf ptr- (send _approach :serialization-length))
   ;; geometry_msgs/Vector3 _surface_center
     (send _surface_center :deserialize buf ptr-) (incf ptr- (send _surface_center :serialization-length))
   ;; std_msgs/Float32 _width
     (send _width :deserialize buf ptr-) (incf ptr- (send _width :serialization-length))
   ;;
   self)
  )

(setf (get agile_grasp::Grasp :md5sum-) "e812ccd1fa0a0ad5be105b582346ad98")
(setf (get agile_grasp::Grasp :datatype-) "agile_grasp/Grasp")
(setf (get agile_grasp::Grasp :definition-)
      "geometry_msgs/Vector3 center
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
")



(provide :agile_grasp/Grasp "e812ccd1fa0a0ad5be105b582346ad98")


