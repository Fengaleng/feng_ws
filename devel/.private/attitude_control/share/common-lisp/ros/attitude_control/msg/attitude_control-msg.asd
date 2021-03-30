
(cl:in-package :asdf)

(defsystem "attitude_control-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "uav_state" :depends-on ("_package_uav_state"))
    (:file "_package_uav_state" :depends-on ("_package"))
  ))