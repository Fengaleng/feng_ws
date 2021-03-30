// Auto-generated. Do not edit!

// (in-package traj_gen.msg)


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

class min_snap_traj {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.position_ref = null;
      this.velocity_ref = null;
      this.accel_ref = null;
      this.yaw_ref = null;
      this.launch_flag = null;
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
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type min_snap_traj
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [position_ref]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.position_ref, buffer, bufferOffset);
    // Serialize message field [velocity_ref]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.velocity_ref, buffer, bufferOffset);
    // Serialize message field [accel_ref]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.accel_ref, buffer, bufferOffset);
    // Check that the constant length array field [yaw_ref] has the right length
    if (obj.yaw_ref.length !== 3) {
      throw new Error('Unable to serialize array field yaw_ref - length must be 3')
    }
    // Serialize message field [yaw_ref]
    bufferOffset = _arraySerializer.float64(obj.yaw_ref, buffer, bufferOffset, 3);
    // Serialize message field [launch_flag]
    bufferOffset = _serializer.bool(obj.launch_flag, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type min_snap_traj
    let len;
    let data = new min_snap_traj(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [position_ref]
    data.position_ref = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [velocity_ref]
    data.velocity_ref = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [accel_ref]
    data.accel_ref = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [yaw_ref]
    data.yaw_ref = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    // Deserialize message field [launch_flag]
    data.launch_flag = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 97;
  }

  static datatype() {
    // Returns string type for a message object
    return 'traj_gen/min_snap_traj';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0d04aef5adebadf0e66bcb0fbc43cb67';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    
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
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new min_snap_traj(null);
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

    return resolved;
    }
};

module.exports = min_snap_traj;
