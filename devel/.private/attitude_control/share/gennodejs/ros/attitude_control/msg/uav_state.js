// Auto-generated. Do not edit!

// (in-package attitude_control.msg)


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

class uav_state {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.position_W = null;
      this.velocity_W = null;
      this.euler_angle = null;
      this.rotation_speed_B = null;
      this.commanded_thrust = null;
      this.moment = null;
      this.position_ref = null;
      this.velocity_ref = null;
      this.accel_ref = null;
      this.yaw_ref = null;
      this.speed = null;
      this.launch_flag = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('position_W')) {
        this.position_W = initObj.position_W
      }
      else {
        this.position_W = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('velocity_W')) {
        this.velocity_W = initObj.velocity_W
      }
      else {
        this.velocity_W = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('euler_angle')) {
        this.euler_angle = initObj.euler_angle
      }
      else {
        this.euler_angle = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('rotation_speed_B')) {
        this.rotation_speed_B = initObj.rotation_speed_B
      }
      else {
        this.rotation_speed_B = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('commanded_thrust')) {
        this.commanded_thrust = initObj.commanded_thrust
      }
      else {
        this.commanded_thrust = 0.0;
      }
      if (initObj.hasOwnProperty('moment')) {
        this.moment = initObj.moment
      }
      else {
        this.moment = new geometry_msgs.msg.Vector3();
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
      if (initObj.hasOwnProperty('yaw_ref')) {
        this.yaw_ref = initObj.yaw_ref
      }
      else {
        this.yaw_ref = 0.0;
      }
      if (initObj.hasOwnProperty('speed')) {
        this.speed = initObj.speed
      }
      else {
        this.speed = 0.0;
      }
      if (initObj.hasOwnProperty('launch_flag')) {
        this.launch_flag = initObj.launch_flag
      }
      else {
        this.launch_flag = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type uav_state
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [position_W]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.position_W, buffer, bufferOffset);
    // Serialize message field [velocity_W]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.velocity_W, buffer, bufferOffset);
    // Serialize message field [euler_angle]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.euler_angle, buffer, bufferOffset);
    // Serialize message field [rotation_speed_B]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.rotation_speed_B, buffer, bufferOffset);
    // Serialize message field [commanded_thrust]
    bufferOffset = _serializer.float64(obj.commanded_thrust, buffer, bufferOffset);
    // Serialize message field [moment]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.moment, buffer, bufferOffset);
    // Serialize message field [position_ref]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.position_ref, buffer, bufferOffset);
    // Serialize message field [velocity_ref]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.velocity_ref, buffer, bufferOffset);
    // Serialize message field [accel_ref]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.accel_ref, buffer, bufferOffset);
    // Serialize message field [yaw_ref]
    bufferOffset = _serializer.float64(obj.yaw_ref, buffer, bufferOffset);
    // Serialize message field [speed]
    bufferOffset = _serializer.float64(obj.speed, buffer, bufferOffset);
    // Serialize message field [launch_flag]
    bufferOffset = _serializer.bool(obj.launch_flag, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type uav_state
    let len;
    let data = new uav_state(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [position_W]
    data.position_W = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [velocity_W]
    data.velocity_W = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [euler_angle]
    data.euler_angle = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [rotation_speed_B]
    data.rotation_speed_B = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [commanded_thrust]
    data.commanded_thrust = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [moment]
    data.moment = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [position_ref]
    data.position_ref = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [velocity_ref]
    data.velocity_ref = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [accel_ref]
    data.accel_ref = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [yaw_ref]
    data.yaw_ref = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [speed]
    data.speed = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [launch_flag]
    data.launch_flag = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 217;
  }

  static datatype() {
    // Returns string type for a message object
    return 'attitude_control/uav_state';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f9313a03a91ff0a9c49a7d3dc8670c6b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    
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
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new uav_state(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.position_W !== undefined) {
      resolved.position_W = geometry_msgs.msg.Vector3.Resolve(msg.position_W)
    }
    else {
      resolved.position_W = new geometry_msgs.msg.Vector3()
    }

    if (msg.velocity_W !== undefined) {
      resolved.velocity_W = geometry_msgs.msg.Vector3.Resolve(msg.velocity_W)
    }
    else {
      resolved.velocity_W = new geometry_msgs.msg.Vector3()
    }

    if (msg.euler_angle !== undefined) {
      resolved.euler_angle = geometry_msgs.msg.Vector3.Resolve(msg.euler_angle)
    }
    else {
      resolved.euler_angle = new geometry_msgs.msg.Vector3()
    }

    if (msg.rotation_speed_B !== undefined) {
      resolved.rotation_speed_B = geometry_msgs.msg.Vector3.Resolve(msg.rotation_speed_B)
    }
    else {
      resolved.rotation_speed_B = new geometry_msgs.msg.Vector3()
    }

    if (msg.commanded_thrust !== undefined) {
      resolved.commanded_thrust = msg.commanded_thrust;
    }
    else {
      resolved.commanded_thrust = 0.0
    }

    if (msg.moment !== undefined) {
      resolved.moment = geometry_msgs.msg.Vector3.Resolve(msg.moment)
    }
    else {
      resolved.moment = new geometry_msgs.msg.Vector3()
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

    if (msg.yaw_ref !== undefined) {
      resolved.yaw_ref = msg.yaw_ref;
    }
    else {
      resolved.yaw_ref = 0.0
    }

    if (msg.speed !== undefined) {
      resolved.speed = msg.speed;
    }
    else {
      resolved.speed = 0.0
    }

    if (msg.launch_flag !== undefined) {
      resolved.launch_flag = msg.launch_flag;
    }
    else {
      resolved.launch_flag = false
    }

    return resolved;
    }
};

module.exports = uav_state;
