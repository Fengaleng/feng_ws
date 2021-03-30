;; Auto-generated. Do not edit!


(when (boundp 'traj_gen::min_snap_traj)
  (if (not (find-package "TRAJ_GEN"))
    (make-package "TRAJ_GEN"))
  (shadow 'min_snap_traj (find-package "TRAJ_GEN")))
(unless (find-package "TRAJ_GEN::MIN_SNAP_TRAJ")
  (make-package "TRAJ_GEN::MIN_SNAP_TRAJ"))

(in-package "ROS")
;;//! \htmlinclude min_snap_traj.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass traj_gen::min_snap_traj
  :super ros::object
  :slots (_header _position_ref _velocity_ref _accel_ref _yaw_ref _launch_flag ))

(defmethod traj_gen::min_snap_traj
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:position_ref __position_ref) (instance geometry_msgs::Vector3 :init))
    ((:velocity_ref __velocity_ref) (instance geometry_msgs::Vector3 :init))
    ((:accel_ref __accel_ref) (instance geometry_msgs::Vector3 :init))
    ((:yaw_ref __yaw_ref) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:launch_flag __launch_flag) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _position_ref __position_ref)
   (setq _velocity_ref __velocity_ref)
   (setq _accel_ref __accel_ref)
   (setq _yaw_ref __yaw_ref)
   (setq _launch_flag __launch_flag)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:position_ref
   (&rest __position_ref)
   (if (keywordp (car __position_ref))
       (send* _position_ref __position_ref)
     (progn
       (if __position_ref (setq _position_ref (car __position_ref)))
       _position_ref)))
  (:velocity_ref
   (&rest __velocity_ref)
   (if (keywordp (car __velocity_ref))
       (send* _velocity_ref __velocity_ref)
     (progn
       (if __velocity_ref (setq _velocity_ref (car __velocity_ref)))
       _velocity_ref)))
  (:accel_ref
   (&rest __accel_ref)
   (if (keywordp (car __accel_ref))
       (send* _accel_ref __accel_ref)
     (progn
       (if __accel_ref (setq _accel_ref (car __accel_ref)))
       _accel_ref)))
  (:yaw_ref
   (&optional __yaw_ref)
   (if __yaw_ref (setq _yaw_ref __yaw_ref)) _yaw_ref)
  (:launch_flag
   (&optional __launch_flag)
   (if __launch_flag (setq _launch_flag __launch_flag)) _launch_flag)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Vector3 _position_ref
    (send _position_ref :serialization-length)
    ;; geometry_msgs/Vector3 _velocity_ref
    (send _velocity_ref :serialization-length)
    ;; geometry_msgs/Vector3 _accel_ref
    (send _accel_ref :serialization-length)
    ;; float64[3] _yaw_ref
    (* 8    3)
    ;; bool _launch_flag
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Vector3 _position_ref
       (send _position_ref :serialize s)
     ;; geometry_msgs/Vector3 _velocity_ref
       (send _velocity_ref :serialize s)
     ;; geometry_msgs/Vector3 _accel_ref
       (send _accel_ref :serialize s)
     ;; float64[3] _yaw_ref
     (dotimes (i 3)
       (sys::poke (elt _yaw_ref i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; bool _launch_flag
       (if _launch_flag (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Vector3 _position_ref
     (send _position_ref :deserialize buf ptr-) (incf ptr- (send _position_ref :serialization-length))
   ;; geometry_msgs/Vector3 _velocity_ref
     (send _velocity_ref :deserialize buf ptr-) (incf ptr- (send _velocity_ref :serialization-length))
   ;; geometry_msgs/Vector3 _accel_ref
     (send _accel_ref :deserialize buf ptr-) (incf ptr- (send _accel_ref :serialization-length))
   ;; float64[3] _yaw_ref
   (dotimes (i (length _yaw_ref))
     (setf (elt _yaw_ref i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; bool _launch_flag
     (setq _launch_flag (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get traj_gen::min_snap_traj :md5sum-) "0d04aef5adebadf0e66bcb0fbc43cb67")
(setf (get traj_gen::min_snap_traj :datatype-) "traj_gen/min_snap_traj")
(setf (get traj_gen::min_snap_traj :definition-)
      "Header header

# references
geometry_msgs/Vector3 position_ref
geometry_msgs/Vector3 velocity_ref
geometry_msgs/Vector3 accel_ref
float64[3] yaw_ref
bool launch_flag

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



(provide :traj_gen/min_snap_traj "0d04aef5adebadf0e66bcb0fbc43cb67")


