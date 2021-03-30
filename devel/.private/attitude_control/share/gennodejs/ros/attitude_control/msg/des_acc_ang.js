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

class des_acc_ang {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.accel_out = null;
      this.ang_vel_out = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('accel_out')) {
        this.accel_out = initObj.accel_out
      }
      else {
        this.accel_out = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('ang_vel_out')) {
        this.ang_vel_out = initObj.ang_vel_out
      }
      else {
        this.ang_vel_out = new geometry_msgs.msg.Vector3();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type des_acc_ang
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [accel_out]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.accel_out, buffer, bufferOffset);
    // Serialize message field [ang_vel_out]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.ang_vel_out, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type des_acc_ang
    let len;
    let data = new des_acc_ang(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [accel_out]
    data.accel_out = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [ang_vel_out]
    data.ang_vel_out = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 48;
  }

  static datatype() {
    // Returns string type for a message object
    return 'attitude_control/des_acc_ang';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f9a93238a0cb64bcae38d4c887a94d2a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    
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
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new des_acc_ang(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.accel_out !== undefined) {
      resolved.accel_out = geometry_msgs.msg.Vector3.Resolve(msg.accel_out)
    }
    else {
      resolved.accel_out = new geometry_msgs.msg.Vector3()
    }

    if (msg.ang_vel_out !== undefined) {
      resolved.ang_vel_out = geometry_msgs.msg.Vector3.Resolve(msg.ang_vel_out)
    }
    else {
      resolved.ang_vel_out = new geometry_msgs.msg.Vector3()
    }

    return resolved;
    }
};

module.exports = des_acc_ang;
