
(cl:in-package :asdf)

(defsystem "agile_grasp-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :agile_grasp-msg
               :geometry_msgs-msg
               :sensor_msgs-msg
)
  :components ((:file "_package")
    (:file "FindGrasps" :depends-on ("_package_FindGrasps"))
    (:file "_package_FindGrasps" :depends-on ("_package"))
  ))