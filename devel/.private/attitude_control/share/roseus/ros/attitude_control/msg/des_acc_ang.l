;; Auto-generated. Do not edit!


(when (boundp 'attitude_control::des_acc_ang)
  (if (not (find-package "ATTITUDE_CONTROL"))
    (make-package "ATTITUDE_CONTROL"))
  (shadow 'des_acc_ang (find-package "ATTITUDE_CONTROL")))
(unless (find-package "ATTITUDE_CONTROL::DES_ACC_ANG")
  (make-package "ATTITUDE_CONTROL::DES_ACC_ANG"))

(in-package "ROS")
;;//! \htmlinclude des_acc_ang.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass attitude_control::des_acc_ang
  :super ros::object
  :slots (_header _accel_out _ang_vel_out ))

(defmethod attitude_control::des_acc_ang
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:accel_out __accel_out) (instance geometry_msgs::Vector3 :init))
    ((:ang_vel_out __ang_vel_out) (instance geometry_msgs::Vector3 :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _accel_out __accel_out)
   (setq _ang_vel_out __ang_vel_out)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:accel_out
   (&rest __accel_out)
   (if (keywordp (car __accel_out))
       (send* _accel_out __accel_out)
     (progn
       (if __accel_out (setq _accel_out (car __accel_out)))
       _accel_out)))
  (:ang_vel_out
   (&rest __ang_vel_out)
   (if (keywordp (car __ang_vel_out))
       (send* _ang_vel_out __ang_vel_out)
     (progn
       (if __ang_vel_out (setq _ang_vel_out (car __ang_vel_out)))
       _ang_vel_out)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Vector3 _accel_out
    (send _accel_out :serialization-length)
    ;; geometry_msgs/Vector3 _ang_vel_out
    (send _ang_vel_out :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Vector3 _accel_out
       (send _accel_out :serialize s)
     ;; geometry_msgs/Vector3 _ang_vel_out
       (send _ang_vel_out :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Vector3 _accel_out
     (send _accel_out :deserialize buf ptr-) (incf ptr- (send _accel_out :serialization-length))
   ;; geometry_msgs/Vector3 _ang_vel_out
     (send _ang_vel_out :deserialize buf ptr-) (incf ptr- (send _ang_vel_out :serialization-length))
   ;;
   self)
  )

(setf (get attitude_control::des_acc_ang :md5sum-) "f9a93238a0cb64bcae38d4c887a94d2a")
(setf (get attitude_control::des_acc_ang :datatype-) "attitude_control/des_acc_ang")
(setf (get attitude_control::des_acc_ang :definition-)
      "Header header

# references
geometry_msgs/Vector3 accel_out
geometry_msgs/Vector3 ang_vel_out

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

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
")



(provide :attitude_control/des_acc_ang "f9a93238a0cb64bcae38d4c887a94d2a")


