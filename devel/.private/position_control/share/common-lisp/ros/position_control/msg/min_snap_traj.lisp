; Auto-generated. Do not edit!


(cl:in-package position_control-msg)


;//! \htmlinclude min_snap_traj.msg.html

(cl:defclass <min_snap_traj> (roslisp-msg-protocol:ros-message)
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
   (yaw_ref
    :reader yaw_ref
    :initarg :yaw_ref
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (launch_flag
    :reader launch_flag
    :initarg :launch_flag
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass min_snap_traj (<min_snap_traj>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <min_snap_traj>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'min_snap_traj)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name position_control-msg:<min_snap_traj> is deprecated: use position_control-msg:min_snap_traj instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <min_snap_traj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader position_control-msg:header-val is deprecated.  Use position_control-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'position_ref-val :lambda-list '(m))
(cl:defmethod position_ref-val ((m <min_snap_traj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader position_control-msg:position_ref-val is deprecated.  Use position_control-msg:position_ref instead.")
  (position_ref m))

(cl:ensure-generic-function 'velocity_ref-val :lambda-list '(m))
(cl:defmethod velocity_ref-val ((m <min_snap_traj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader position_control-msg:velocity_ref-val is deprecated.  Use position_control-msg:velocity_ref instead.")
  (velocity_ref m))

(cl:ensure-generic-function 'accel_ref-val :lambda-list '(m))
(cl:defmethod accel_ref-val ((m <min_snap_traj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader position_control-msg:accel_ref-val is deprecated.  Use position_control-msg:accel_ref instead.")
  (accel_ref m))

(cl:ensure-generic-function 'yaw_ref-val :lambda-list '(m))
(cl:defmethod yaw_ref-val ((m <min_snap_traj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader position_control-msg:yaw_ref-val is deprecated.  Use position_control-msg:yaw_ref instead.")
  (yaw_ref m))

(cl:ensure-generic-function 'launch_flag-val :lambda-list '(m))
(cl:defmethod launch_flag-val ((m <min_snap_traj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader position_control-msg:launch_flag-val is deprecated.  Use position_control-msg:launch_flag instead.")
  (launch_flag m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <min_snap_traj>) ostream)
  "Serializes a message object of type '<min_snap_traj>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position_ref) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'velocity_ref) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'accel_ref) ostream)
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
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <min_snap_traj>) istream)
  "Deserializes a message object of type '<min_snap_traj>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position_ref) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'velocity_ref) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'accel_ref) istream)
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
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<min_snap_traj>)))
  "Returns string type for a message object of type '<min_snap_traj>"
  "position_control/min_snap_traj")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'min_snap_traj)))
  "Returns string type for a message object of type 'min_snap_traj"
  "position_control/min_snap_traj")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<min_snap_traj>)))
  "Returns md5sum for a message object of type '<min_snap_traj>"
  "0d04aef5adebadf0e66bcb0fbc43cb67")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'min_snap_traj)))
  "Returns md5sum for a message object of type 'min_snap_traj"
  "0d04aef5adebadf0e66bcb0fbc43cb67")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<min_snap_traj>)))
  "Returns full string definition for message of type '<min_snap_traj>"
  (cl:format cl:nil "Header header~%~%# references~%geometry_msgs/Vector3 position_ref~%geometry_msgs/Vector3 velocity_ref~%geometry_msgs/Vector3 accel_ref~%float64[3] yaw_ref~%bool launch_flag~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'min_snap_traj)))
  "Returns full string definition for message of type 'min_snap_traj"
  (cl:format cl:nil "Header header~%~%# references~%geometry_msgs/Vector3 position_ref~%geometry_msgs/Vector3 velocity_ref~%geometry_msgs/Vector3 accel_ref~%float64[3] yaw_ref~%bool launch_flag~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <min_snap_traj>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position_ref))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'velocity_ref))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'accel_ref))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'yaw_ref) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <min_snap_traj>))
  "Converts a ROS message object to a list"
  (cl:list 'min_snap_traj
    (cl:cons ':header (header msg))
    (cl:cons ':position_ref (position_ref msg))
    (cl:cons ':velocity_ref (velocity_ref msg))
    (cl:cons ':accel_ref (accel_ref msg))
    (cl:cons ':yaw_ref (yaw_ref msg))
    (cl:cons ':launch_flag (launch_flag msg))
))
