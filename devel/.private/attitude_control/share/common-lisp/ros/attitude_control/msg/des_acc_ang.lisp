; Auto-generated. Do not edit!


(cl:in-package attitude_control-msg)


;//! \htmlinclude des_acc_ang.msg.html

(cl:defclass <des_acc_ang> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (accel_out
    :reader accel_out
    :initarg :accel_out
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (ang_vel_out
    :reader ang_vel_out
    :initarg :ang_vel_out
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3)))
)

(cl:defclass des_acc_ang (<des_acc_ang>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <des_acc_ang>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'des_acc_ang)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name attitude_control-msg:<des_acc_ang> is deprecated: use attitude_control-msg:des_acc_ang instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <des_acc_ang>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader attitude_control-msg:header-val is deprecated.  Use attitude_control-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'accel_out-val :lambda-list '(m))
(cl:defmethod accel_out-val ((m <des_acc_ang>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader attitude_control-msg:accel_out-val is deprecated.  Use attitude_control-msg:accel_out instead.")
  (accel_out m))

(cl:ensure-generic-function 'ang_vel_out-val :lambda-list '(m))
(cl:defmethod ang_vel_out-val ((m <des_acc_ang>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader attitude_control-msg:ang_vel_out-val is deprecated.  Use attitude_control-msg:ang_vel_out instead.")
  (ang_vel_out m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <des_acc_ang>) ostream)
  "Serializes a message object of type '<des_acc_ang>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'accel_out) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'ang_vel_out) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <des_acc_ang>) istream)
  "Deserializes a message object of type '<des_acc_ang>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'accel_out) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'ang_vel_out) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<des_acc_ang>)))
  "Returns string type for a message object of type '<des_acc_ang>"
  "attitude_control/des_acc_ang")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'des_acc_ang)))
  "Returns string type for a message object of type 'des_acc_ang"
  "attitude_control/des_acc_ang")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<des_acc_ang>)))
  "Returns md5sum for a message object of type '<des_acc_ang>"
  "f9a93238a0cb64bcae38d4c887a94d2a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'des_acc_ang)))
  "Returns md5sum for a message object of type 'des_acc_ang"
  "f9a93238a0cb64bcae38d4c887a94d2a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<des_acc_ang>)))
  "Returns full string definition for message of type '<des_acc_ang>"
  (cl:format cl:nil "Header header~%~%# references~%geometry_msgs/Vector3 accel_out~%geometry_msgs/Vector3 ang_vel_out~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'des_acc_ang)))
  "Returns full string definition for message of type 'des_acc_ang"
  (cl:format cl:nil "Header header~%~%# references~%geometry_msgs/Vector3 accel_out~%geometry_msgs/Vector3 ang_vel_out~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <des_acc_ang>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'accel_out))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'ang_vel_out))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <des_acc_ang>))
  "Converts a ROS message object to a list"
  (cl:list 'des_acc_ang
    (cl:cons ':header (header msg))
    (cl:cons ':accel_out (accel_out msg))
    (cl:cons ':ang_vel_out (ang_vel_out msg))
))
