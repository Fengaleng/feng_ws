;; Auto-generated. Do not edit!


(when (boundp 'attitude_control::uav_state)
  (if (not (find-package "ATTITUDE_CONTROL"))
    (make-package "ATTITUDE_CONTROL"))
  (shadow 'uav_state (find-package "ATTITUDE_CONTROL")))
(unless (find-package "ATTITUDE_CONTROL::UAV_STATE")
  (make-package "ATTITUDE_CONTROL::UAV_STATE"))

(in-package "ROS")
;;//! \htmlinclude uav_state.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass attitude_control::uav_state
  :super ros::object
  :slots (_header _position_W _velocity_W _euler_angle _rotation_speed_B _commanded_thrust _moment _position_ref _velocity_ref _accel_ref _yaw_ref _speed _launch_flag ))

(defmethod attitude_control::uav_state
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:position_W __position_W) (instance geometry_msgs::Vector3 :init))
    ((:velocity_W __velocity_W) (instance geometry_msgs::Vector3 :init))
    ((:euler_angle __euler_angle) (instance geometry_msgs::Vector3 :init))
    ((:rotation_speed_B __rotation_speed_B) (instance geometry_msgs::Vector3 :init))
    ((:commanded_thrust __commanded_thrust) 0.0)
    ((:moment __moment) (instance geometry_msgs::Vector3 :init))
    ((:position_ref __position_ref) (instance geometry_msgs::Vector3 :init))
    ((:velocity_ref __velocity_ref) (instance geometry_msgs::Vector3 :init))
    ((:accel_ref __accel_ref) (instance geometry_msgs::Vector3 :init))
    ((:yaw_ref __yaw_ref) 0.0)
    ((:speed __speed) 0.0)
    ((:launch_flag __launch_flag) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _position_W __position_W)
   (setq _velocity_W __velocity_W)
   (setq _euler_angle __euler_angle)
   (setq _rotation_speed_B __rotation_speed_B)
   (setq _commanded_thrust (float __commanded_thrust))
   (setq _moment __moment)
   (setq _position_ref __position_ref)
   (setq _velocity_ref __velocity_ref)
   (setq _accel_ref __accel_ref)
   (setq _yaw_ref (float __yaw_ref))
   (setq _speed (float __speed))
   (setq _launch_flag __launch_flag)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:position_W
   (&rest __position_W)
   (if (keywordp (car __position_W))
       (send* _position_W __position_W)
     (progn
       (if __position_W (setq _position_W (car __position_W)))
       _position_W)))
  (:velocity_W
   (&rest __velocity_W)
   (if (keywordp (car __velocity_W))
       (send* _velocity_W __velocity_W)
     (progn
       (if __velocity_W (setq _velocity_W (car __velocity_W)))
       _velocity_W)))
  (:euler_angle
   (&rest __euler_angle)
   (if (keywordp (car __euler_angle))
       (send* _euler_angle __euler_angle)
     (progn
       (if __euler_angle (setq _euler_angle (car __euler_angle)))
       _euler_angle)))
  (:rotation_speed_B
   (&rest __rotation_speed_B)
   (if (keywordp (car __rotation_speed_B))
       (send* _rotation_speed_B __rotation_speed_B)
     (progn
       (if __rotation_speed_B (setq _rotation_speed_B (car __rotation_speed_B)))
       _rotation_speed_B)))
  (:commanded_thrust
   (&optional __commanded_thrust)
   (if __commanded_thrust (setq _commanded_thrust __commanded_thrust)) _commanded_thrust)
  (:moment
   (&rest __moment)
   (if (keywordp (car __moment))
       (send* _moment __moment)
     (progn
       (if __moment (setq _moment (car __moment)))
       _moment)))
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
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:launch_flag
   (&optional __launch_flag)
   (if __launch_flag (setq _launch_flag __launch_flag)) _launch_flag)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Vector3 _position_W
    (send _position_W :serialization-length)
    ;; geometry_msgs/Vector3 _velocity_W
    (send _velocity_W :serialization-length)
    ;; geometry_msgs/Vector3 _euler_angle
    (send _euler_angle :serialization-length)
    ;; geometry_msgs/Vector3 _rotation_speed_B
    (send _rotation_speed_B :serialization-length)
    ;; float64 _commanded_thrust
    8
    ;; geometry_msgs/Vector3 _moment
    (send _moment :serialization-length)
    ;; geometry_msgs/Vector3 _position_ref
    (send _position_ref :serialization-length)
    ;; geometry_msgs/Vector3 _velocity_ref
    (send _velocity_ref :serialization-length)
    ;; geometry_msgs/Vector3 _accel_ref
    (send _accel_ref :serialization-length)
    ;; float64 _yaw_ref
    8
    ;; float64 _speed
    8
    ;; bool _launch_flag
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Vector3 _position_W
       (send _position_W :serialize s)
     ;; geometry_msgs/Vector3 _velocity_W
       (send _velocity_W :serialize s)
     ;; geometry_msgs/Vector3 _euler_angle
       (send _euler_angle :serialize s)
     ;; geometry_msgs/Vector3 _rotation_speed_B
       (send _rotation_speed_B :serialize s)
     ;; float64 _commanded_thrust
       (sys::poke _commanded_thrust (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; geometry_msgs/Vector3 _moment
       (send _moment :serialize s)
     ;; geometry_msgs/Vector3 _position_ref
       (send _position_ref :serialize s)
     ;; geometry_msgs/Vector3 _velocity_ref
       (send _velocity_ref :serialize s)
     ;; geometry_msgs/Vector3 _accel_ref
       (send _accel_ref :serialize s)
     ;; float64 _yaw_ref
       (sys::poke _yaw_ref (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _speed
       (sys::poke _speed (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bool _launch_flag
       (if _launch_flag (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Vector3 _position_W
     (send _position_W :deserialize buf ptr-) (incf ptr- (send _position_W :serialization-length))
   ;; geometry_msgs/Vector3 _velocity_W
     (send _velocity_W :deserialize buf ptr-) (incf ptr- (send _velocity_W :serialization-length))
   ;; geometry_msgs/Vector3 _euler_angle
     (send _euler_angle :deserialize buf ptr-) (incf ptr- (send _euler_angle :serialization-length))
   ;; geometry_msgs/Vector3 _rotation_speed_B
     (send _rotation_speed_B :deserialize buf ptr-) (incf ptr- (send _rotation_speed_B :serialization-length))
   ;; float64 _commanded_thrust
     (setq _commanded_thrust (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; geometry_msgs/Vector3 _moment
     (send _moment :deserialize buf ptr-) (incf ptr- (send _moment :serialization-length))
   ;; geometry_msgs/Vector3 _position_ref
     (send _position_ref :deserialize buf ptr-) (incf ptr- (send _position_ref :serialization-length))
   ;; geometry_msgs/Vector3 _velocity_ref
     (send _velocity_ref :deserialize buf ptr-) (incf ptr- (send _velocity_ref :serialization-length))
   ;; geometry_msgs/Vector3 _accel_ref
     (send _accel_ref :deserialize buf ptr-) (incf ptr- (send _accel_ref :serialization-length))
   ;; float64 _yaw_ref
     (setq _yaw_ref (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _speed
     (setq _speed (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool _launch_flag
     (setq _launch_flag (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get attitude_control::uav_state :md5sum-) "f9313a03a91ff0a9c49a7d3dc8670c6b")
(setf (get attitude_control::uav_state :datatype-) "attitude_control/uav_state")
(setf (get attitude_control::uav_state :definition-)
      "Header header

# state
geometry_msgs/Vector3 position_W
geometry_msgs/Vector3 velocity_W
geometry_msgs/Vector3 euler_angle
geometry_msgs/Vector3 rotation_speed_B

# control
float64 commanded_thrust
geometry_msgs/Vector3 moment

# ref trajectory
geometry_msgs/Vector3 position_ref
geometry_msgs/Vector3 velocity_ref
geometry_msgs/Vector3 accel_ref
float64 yaw_ref
float64 speed
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



(provide :attitude_control/uav_state "f9313a03a91ff0a9c49a7d3dc8670c6b")


