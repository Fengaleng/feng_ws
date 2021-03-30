; Auto-generated. Do not edit!


(cl:in-package target_detection-msg)


;//! \htmlinclude target_dect_msg.msg.html

(cl:defclass <target_dect_msg> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
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
   (jerk_ref
    :reader jerk_ref
    :initarg :jerk_ref
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (snap_ref
    :reader snap_ref
    :initarg :snap_ref
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (yaw_ref
    :reader yaw_ref
    :initarg :yaw_ref
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (launch_flag
    :reader launch_flag
    :initarg :launch_flag
    :type cl:boolean
    :initform cl:nil)
   (speed
    :reader speed
    :initarg :speed
    :type cl:float
    :initform 0.0)
   (collide_state
    :reader collide_state
    :initarg :collide_state
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass target_dect_msg (<target_dect_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <target_dect_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'target_dect_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name target_detection-msg:<target_dect_msg> is deprecated: use target_detection-msg:target_dect_msg instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <target_dect_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader target_detection-msg:header-val is deprecated.  Use target_detection-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'position_ref-val :lambda-list '(m))
(cl:defmethod position_ref-val ((m <target_dect_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader target_detection-msg:position_ref-val is deprecated.  Use target_detection-msg:position_ref instead.")
  (position_ref m))

(cl:ensure-generic-function 'velocity_ref-val :lambda-list '(m))
(cl:defmethod velocity_ref-val ((m <target_dect_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader target_detection-msg:velocity_ref-val is deprecated.  Use target_detection-msg:velocity_ref instead.")
  (velocity_ref m))

(cl:ensure-generic-function 'accel_ref-val :lambda-list '(m))
(cl:defmethod accel_ref-val ((m <target_dect_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader target_detection-msg:accel_ref-val is deprecated.  Use target_detection-msg:accel_ref instead.")
  (accel_ref m))

(cl:ensure-generic-function 'jerk_ref-val :lambda-list '(m))
(cl:defmethod jerk_ref-val ((m <target_dect_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader target_detection-msg:jerk_ref-val is deprecated.  Use target_detection-msg:jerk_ref instead.")
  (jerk_ref m))

(cl:ensure-generic-function 'snap_ref-val :lambda-list '(m))
(cl:defmethod snap_ref-val ((m <target_dect_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader target_detection-msg:snap_ref-val is deprecated.  Use target_detection-msg:snap_ref instead.")
  (snap_ref m))

(cl:ensure-generic-function 'yaw_ref-val :lambda-list '(m))
(cl:defmethod yaw_ref-val ((m <target_dect_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader target_detection-msg:yaw_ref-val is deprecated.  Use target_detection-msg:yaw_ref instead.")
  (yaw_ref m))

(cl:ensure-generic-function 'launch_flag-val :lambda-list '(m))
(cl:defmethod launch_flag-val ((m <target_dect_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader target_detection-msg:launch_flag-val is deprecated.  Use target_detection-msg:launch_flag instead.")
  (launch_flag m))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <target_dect_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader target_detection-msg:speed-val is deprecated.  Use target_detection-msg:speed instead.")
  (speed m))

(cl:ensure-generic-function 'collide_state-val :lambda-list '(m))
(cl:defmethod collide_state-val ((m <target_dect_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader target_detection-msg:collide_state-val is deprecated.  Use target_detection-msg:collide_state instead.")
  (collide_state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <target_dect_msg>) ostream)
  "Serializes a message object of type '<target_dect_msg>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position_ref) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'velocity_ref) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'accel_ref) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'jerk_ref) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'snap_ref) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'yaw_ref))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'launch_flag) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'collide_state) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <target_dect_msg>) istream)
  "Deserializes a message object of type '<target_dect_msg>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position_ref) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'velocity_ref) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'accel_ref) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'jerk_ref) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'snap_ref) istream)
  (cl:setf (cl:slot-value msg 'yaw_ref) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'yaw_ref)))
    (cl:dotimes (i 3)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
    (cl:setf (cl:slot-value msg 'launch_flag) (cl:not (cl:zerop (cl:read-byte istream))))
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
    (cl:setf (cl:slot-value msg 'collide_state) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<target_dect_msg>)))
  "Returns string type for a message object of type '<target_dect_msg>"
  "target_detection/target_dect_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'target_dect_msg)))
  "Returns string type for a message object of type 'target_dect_msg"
  "target_detection/target_dect_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<target_dect_msg>)))
  "Returns md5sum for a message object of type '<target_dect_msg>"
  "2a847c4ada344c2340bcd2fb12cdd132")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'target_dect_msg)))
  "Returns md5sum for a message object of type 'target_dect_msg"
  "2a847c4ada344c2340bcd2fb12cdd132")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<target_dect_msg>)))
  "Returns full string definition for message of type '<target_dect_msg>"
  (cl:format cl:nil "Header header~%~%# references~%geometry_msgs/Vector3 position_ref~%geometry_msgs/Vector3 velocity_ref~%geometry_msgs/Vector3 accel_ref~%geometry_msgs/Vector3 jerk_ref~%geometry_msgs/Vector3 snap_ref~%float64[3] yaw_ref~%bool launch_flag~%float64 speed~%bool collide_state~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'target_dect_msg)))
  "Returns full string definition for message of type 'target_dect_msg"
  (cl:format cl:nil "Header header~%~%# references~%geometry_msgs/Vector3 position_ref~%geometry_msgs/Vector3 velocity_ref~%geometry_msgs/Vector3 accel_ref~%geometry_msgs/Vector3 jerk_ref~%geometry_msgs/Vector3 snap_ref~%float64[3] yaw_ref~%bool launch_flag~%float64 speed~%bool collide_state~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <target_dect_msg>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position_ref))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'velocity_ref))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'accel_ref))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'jerk_ref))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'snap_ref))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'yaw_ref) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     1
     8
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <target_dect_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'target_dect_msg
    (cl:cons ':header (header msg))
    (cl:cons ':position_ref (position_ref msg))
    (cl:cons ':velocity_ref (velocity_ref msg))
    (cl:cons ':accel_ref (accel_ref msg))
    (cl:cons ':jerk_ref (jerk_ref msg))
    (cl:cons ':snap_ref (snap_ref msg))
    (cl:cons ':yaw_ref (yaw_ref msg))
    (cl:cons ':launch_flag (launch_flag msg))
    (cl:cons ':speed (speed msg))
    (cl:cons ':collide_state (collide_state msg))
))
