// Auto-generated. Do not edit!

// (in-package position_control.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class target_dect_msg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.position_ref = null;
      this.velocity_ref = null;
      this.accel_ref = null;
      this.jerk_ref = null;
      this.snap_ref = null;
      this.yaw_ref = null;
      this.launch_flag = null;
      this.speed = null;
      this.collide_state = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('position_ref')) {
        this.position_ref = initObj.position_ref
      }
      else {
        this.position_ref = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('velocity_ref')) {
        this.velocity_ref = initObj.velocity_ref
      }
      else {
        this.velocity_ref = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('accel_ref')) {
        this.accel_ref = initObj.accel_ref
      }
      else {
        this.accel_ref = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('jerk_ref')) {
        this.jerk_ref = initObj.jerk_ref
      }
      else {
        this.jerk_ref = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('snap_ref')) {
        this.snap_ref = initObj.snap_ref
      }
      else {
        this.snap_ref = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('yaw_ref')) {
        this.yaw_ref = initObj.yaw_ref
      }
      else {
        this.yaw_ref = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('launch_flag')) {
        this.launch_flag = initObj.launch_flag
      }
      else {
        this.launch_flag = false;
      }
      if (initObj.hasOwnProperty('speed')) {
        this.speed = initObj.speed
      }
      else {
        this.speed = 0.0;
      }
      if (initObj.hasOwnProperty('collide_state')) {
        this.collide_state = initObj.collide_state
      }
      else {
        this.collide_state = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type target_dect_msg
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [position_ref]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.position_ref, buffer, bufferOffset);
    // Serialize message field [velocity_ref]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.velocity_ref, buffer, bufferOffset);
    // Serialize message field [accel_ref]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.accel_ref, buffer, bufferOffset);
    // Serialize message field [jerk_ref]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.jerk_ref, buffer, bufferOffset);
    // Serialize message field [snap_ref]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.snap_ref, buffer, bufferOffset);
    // Check that the constant length array field [yaw_ref] has the right length
    if (obj.yaw_ref.length !== 3) {
      throw new Error('Unable to serialize array field yaw_ref - length must be 3')
    }
    // Serialize message field [yaw_ref]
    bufferOffset = _arraySerializer.float64(obj.yaw_ref, buffer, bufferOffset, 3);
    // Serialize message field [launch_flag]
    bufferOffset = _serializer.bool(obj.launch_flag, buffer, bufferOffset);
    // Serialize message field [speed]
    bufferOffset = _serializer.float64(obj.speed, buffer, bufferOffset);
    // Serialize message field [collide_state]
    bufferOffset = _serializer.bool(obj.collide_state, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type target_dect_msg
    let len;
    let data = new target_dect_msg(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [position_ref]
    data.position_ref = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [velocity_ref]
    data.velocity_ref = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [accel_ref]
    data.accel_ref = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [jerk_ref]
    data.jerk_ref = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [snap_ref]
    data.snap_ref = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [yaw_ref]
    data.yaw_ref = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    // Deserialize message field [launch_flag]
    data.launch_flag = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [speed]
    data.speed = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [collide_state]
    data.collide_state = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 154;
  }

  static datatype() {
    // Returns string type for a message object
    return 'position_control/target_dect_msg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2a847c4ada344c2340bcd2fb12cdd132';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    
    # references
    geometry_msgs/Vector3 position_ref
    geometry_msgs/Vector3 velocity_ref
    geometry_msgs/Vector3 accel_ref
    geometry_msgs/Vector3 jerk_ref
    geometry_msgs/Vector3 snap_ref
    float64[3] yaw_ref
    bool launch_flag
    float64 speed
    bool collide_state
    
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
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new target_dect_msg(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.position_ref !== undefined) {
      resolved.position_ref = geometry_msgs.msg.Vector3.Resolve(msg.position_ref)
    }
    else {
      resolved.position_ref = new geometry_msgs.msg.Vector3()
    }

    if (msg.velocity_ref !== undefined) {
      resolved.velocity_ref = geometry_msgs.msg.Vector3.Resolve(msg.velocity_ref)
    }
    else {
      resolved.velocity_ref = new geometry_msgs.msg.Vector3()
    }

    if (msg.accel_ref !== undefined) {
      resolved.accel_ref = geometry_msgs.msg.Vector3.Resolve(msg.accel_ref)
    }
    else {
      resolved.accel_ref = new geometry_msgs.msg.Vector3()
    }

    if (msg.jerk_ref !== undefined) {
      resolved.jerk_ref = geometry_msgs.msg.Vector3.Resolve(msg.jerk_ref)
    }
    else {
      resolved.jerk_ref = new geometry_msgs.msg.Vector3()
    }

    if (msg.snap_ref !== undefined) {
      resolved.snap_ref = geometry_msgs.msg.Vector3.Resolve(msg.snap_ref)
    }
    else {
      resolved.snap_ref = new geometry_msgs.msg.Vector3()
    }

    if (msg.yaw_ref !== undefined) {
      resolved.yaw_ref = msg.yaw_ref;
    }
    else {
      resolved.yaw_ref = new Array(3).fill(0)
    }

    if (msg.launch_flag !== undefined) {
      resolved.launch_flag = msg.launch_flag;
    }
    else {
      resolved.launch_flag = false
    }

    if (msg.speed !== undefined) {
      resolved.speed = msg.speed;
    }
    else {
      resolved.speed = 0.0
    }

    if (msg.collide_state !== undefined) {
      resolved.collide_state = msg.collide_state;
    }
    else {
      resolved.collide_state = false
    }

    return resolved;
    }
};

module.exports = target_dect_msg;
