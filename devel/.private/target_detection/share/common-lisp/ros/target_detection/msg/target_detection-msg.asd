
(cl:in-package :asdf)

(defsystem "target_detection-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "target_dect_msg" :depends-on ("_package_target_dect_msg"))
    (:file "_package_target_dect_msg" :depends-on ("_package"))
  ))