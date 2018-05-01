
(cl:in-package :asdf)

(defsystem "agile_grasp-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :sensor_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "CloudSized" :depends-on ("_package_CloudSized"))
    (:file "_package_CloudSized" :depends-on ("_package"))
    (:file "Grasp" :depends-on ("_package_Grasp"))
    (:file "_package_Grasp" :depends-on ("_package"))
    (:file "Grasps" :depends-on ("_package_Grasps"))
    (:file "_package_Grasps" :depends-on ("_package"))
  ))