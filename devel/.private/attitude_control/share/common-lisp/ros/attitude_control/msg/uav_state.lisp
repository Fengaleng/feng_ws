; Auto-generated. Do not edit!


(cl:in-package attitude_control-msg)


;//! \htmlinclude uav_state.msg.html

(cl:defclass <uav_state> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (position_W
    :reader position_W
    :initarg :position_W
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (velocity_W
    :reader velocity_W
    :initarg :velocity_W
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (euler_angle
    :reader euler_angle
    :initarg :euler_angle
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (rotation_speed_B
    :reader rotation_speed_B
    :initarg :rotation_speed_B
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (commanded_thrust
    :reader commanded_thrust
    :initarg :commanded_thrust
    :type cl:float
    :initform 0.0)
   (moment
    :reader moment
    :initarg :moment
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (position_ref
    :reader position_ref
    :initarg :position_ref
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (velocity_ref
    :reader velocity_ref
    :initarg :velocity_ref
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (accel_ref
    :reader accel_ref
    :initarg :accel_ref
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (yaw_ref
    :reader yaw_ref
    :initarg :yaw_ref
    :type cl:float
    :initform 0.0)
   (speed
    :reader speed
    :initarg :speed
    :type cl:float
    :initform 0.0)
   (launch_flag
    :reader launch_flag
    :initarg :launch_flag
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass uav_state (<uav_state>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <uav_state>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'uav_state)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name attitude_control-msg:<uav_state> is deprecated: use attitude_control-msg:uav_state instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <uav_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader attitude_control-msg:header-val is deprecated.  Use attitude_control-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'position_W-val :lambda-list '(m))
(cl:defmethod position_W-val ((m <uav_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader attitude_control-msg:position_W-val is deprecated.  Use attitude_control-msg:position_W instead.")
  (position_W m))

(cl:ensure-generic-function 'velocity_W-val :lambda-list '(m))
(cl:defmethod velocity_W-val ((m <uav_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader attitude_control-msg:velocity_W-val is deprecated.  Use attitude_control-msg:velocity_W instead.")
  (velocity_W m))

(cl:ensure-generic-function 'euler_angle-val :lambda-list '(m))
(cl:defmethod euler_angle-val ((m <uav_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader attitude_control-msg:euler_angle-val is deprecated.  Use attitude_control-msg:euler_angle instead.")
  (euler_angle m))

(cl:ensure-generic-function 'rotation_speed_B-val :lambda-list '(m))
(cl:defmethod rotation_speed_B-val ((m <uav_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader attitude_control-msg:rotation_speed_B-val is deprecated.  Use attitude_control-msg:rotation_speed_B instead.")
  (rotation_speed_B m))

(cl:ensure-generic-function 'commanded_thrust-val :lambda-list '(m))
(cl:defmethod commanded_thrust-val ((m <uav_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader attitude_control-msg:commanded_thrust-val is deprecated.  Use attitude_control-msg:commanded_thrust instead.")
  (commanded_thrust m))

(cl:ensure-generic-function 'moment-val :lambda-list '(m))
(cl:defmethod moment-val ((m <uav_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader attitude_control-msg:moment-val is deprecated.  Use attitude_control-msg:moment instead.")
  (moment m))

(cl:ensure-generic-function 'position_ref-val :lambda-list '(m))
(cl:defmethod position_ref-val ((m <uav_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader attitude_control-msg:position_ref-val is deprecated.  Use attitude_control-msg:position_ref instead.")
  (position_ref m))

(cl:ensure-generic-function 'velocity_ref-val :lambda-list '(m))
(cl:defmethod velocity_ref-val ((m <uav_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader attitude_control-msg:velocity_ref-val is deprecated.  Use attitude_control-msg:velocity_ref instead.")
  (velocity_ref m))

(cl:ensure-generic-function 'accel_ref-val :lambda-list '(m))
(cl:defmethod accel_ref-val ((m <uav_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader attitude_control-msg:accel_ref-val is deprecated.  Use attitude_control-msg:accel_ref instead.")
  (accel_ref m))

(cl:ensure-generic-function 'yaw_ref-val :lambda-list '(m))
(cl:defmethod yaw_ref-val ((m <uav_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader attitude_control-msg:yaw_ref-val is deprecated.  Use attitude_control-msg:yaw_ref instead.")
  (yaw_ref m))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <uav_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader attitude_control-msg:speed-val is deprecated.  Use attitude_control-msg:speed instead.")
  (speed m))

(cl:ensure-generic-function 'launch_flag-val :lambda-list '(m))
(cl:defmethod launch_flag-val ((m <uav_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader attitude_control-msg:launch_flag-val is deprecated.  Use attitude_control-msg:launch_flag instead.")
  (launch_flag m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <uav_state>) ostream)
  "Serializes a message object of type '<uav_state>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position_W) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'velocity_W) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'euler_angle) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'rotation_speed_B) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'commanded_thrust))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'moment) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position_ref) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'velocity_ref) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'accel_ref) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'yaw_ref))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'launch_flag) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <uav_state>) istream)
  "Deserializes a message object of type '<uav_state>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position_W) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'velocity_W) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'euler_angle) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'rotation_speed_B) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'commanded_thrust) (roslisp-utils:decode-double-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'moment) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position_ref) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'velocity_ref) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'accel_ref) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yaw_ref) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'speed) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:slot-value msg 'launch_flag) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<uav_state>)))
  "Returns string type for a message object of type '<uav_state>"
  "attitude_control/uav_state")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'uav_state)))
  "Returns string type for a message object of type 'uav_state"
  "attitude_control/uav_state")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<uav_state>)))
  "Returns md5sum for a message object of type '<uav_state>"
  "f9313a03a91ff0a9c49a7d3dc8670c6b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'uav_state)))
  "Returns md5sum for a message object of type 'uav_state"
  "f9313a03a91ff0a9c49a7d3dc8670c6b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<uav_state>)))
  "Returns full string definition for message of type '<uav_state>"
  (cl:format cl:nil "Header header~%~%# state~%geometry_msgs/Vector3 position_W~%geometry_msgs/Vector3 velocity_W~%geometry_msgs/Vector3 euler_angle~%geometry_msgs/Vector3 rotation_speed_B~%~%# control~%float64 commanded_thrust~%geometry_msgs/Vector3 moment~%~%# ref trajectory~%geometry_msgs/Vector3 position_ref~%geometry_msgs/Vector3 velocity_ref~%geometry_msgs/Vector3 accel_ref~%float64 yaw_ref~%float64 speed~%bool launch_flag~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'uav_state)))
  "Returns full string definition for message of type 'uav_state"
  (cl:format cl:nil "Header header~%~%# state~%geometry_msgs/Vector3 position_W~%geometry_msgs/Vector3 velocity_W~%geometry_msgs/Vector3 euler_angle~%geometry_msgs/Vector3 rotation_speed_B~%~%# control~%float64 commanded_thrust~%geometry_msgs/Vector3 moment~%~%# ref trajectory~%geometry_msgs/Vector3 position_ref~%geometry_msgs/Vector3 velocity_ref~%geometry_msgs/Vector3 accel_ref~%float64 yaw_ref~%float64 speed~%bool launch_flag~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <uav_state>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position_W))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'velocity_W))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'euler_angle))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'rotation_speed_B))
     8
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'moment))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position_ref))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'velocity_ref))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'accel_ref))
     8
     8
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <uav_state>))
  "Converts a ROS message object to a list"
  (cl:list 'uav_state
    (cl:cons ':header (header msg))
    (cl:cons ':position_W (position_W msg))
    (cl:cons ':velocity_W (velocity_W msg))
    (cl:cons ':euler_angle (euler_angle msg))
    (cl:cons ':rotation_speed_B (rotation_speed_B msg))
    (cl:cons ':commanded_thrust (commanded_thrust msg))
    (cl:cons ':moment (moment msg))
    (cl:cons ':position_ref (position_ref msg))
    (cl:cons ':velocity_ref (velocity_ref msg))
    (cl:cons ':accel_ref (accel_ref msg))
    (cl:cons ':yaw_ref (yaw_ref msg))
    (cl:cons ':speed (speed msg))
    (cl:cons ':launch_flag (launch_flag msg))
))
