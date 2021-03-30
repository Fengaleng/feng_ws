
(cl:in-package :asdf)

(defsystem "traj_gen-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "min_snap_traj" :depends-on ("_package_min_snap_traj"))
    (:file "_package_min_snap_traj" :depends-on ("_package"))
  ))