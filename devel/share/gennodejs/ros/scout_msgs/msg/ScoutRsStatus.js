// Auto-generated. Do not edit!

// (in-package scout_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class ScoutRsStatus {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.stick_left_h = null;
      this.stick_left_v = null;
      this.stick_right_h = null;
      this.stick_right_v = null;
      this.swa = null;
      this.swb = null;
      this.swc = null;
      this.swd = null;
      this.var_a = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('stick_left_h')) {
        this.stick_left_h = initObj.stick_left_h
      }
      else {
        this.stick_left_h = 0;
      }
      if (initObj.hasOwnProperty('stick_left_v')) {
        this.stick_left_v = initObj.stick_left_v
      }
      else {
        this.stick_left_v = 0;
      }
      if (initObj.hasOwnProperty('stick_right_h')) {
        this.stick_right_h = initObj.stick_right_h
      }
      else {
        this.stick_right_h = 0;
      }
      if (initObj.hasOwnProperty('stick_right_v')) {
        this.stick_right_v = initObj.stick_right_v
      }
      else {
        this.stick_right_v = 0;
      }
      if (initObj.hasOwnProperty('swa')) {
        this.swa = initObj.swa
      }
      else {
        this.swa = 0;
      }
      if (initObj.hasOwnProperty('swb')) {
        this.swb = initObj.swb
      }
      else {
        this.swb = 0;
      }
      if (initObj.hasOwnProperty('swc')) {
        this.swc = initObj.swc
      }
      else {
        this.swc = 0;
      }
      if (initObj.hasOwnProperty('swd')) {
        this.swd = initObj.swd
      }
      else {
        this.swd = 0;
      }
      if (initObj.hasOwnProperty('var_a')) {
        this.var_a = initObj.var_a
      }
      else {
        this.var_a = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ScoutRsStatus
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [stick_left_h]
    bufferOffset = _serializer.int8(obj.stick_left_h, buffer, bufferOffset);
    // Serialize message field [stick_left_v]
    bufferOffset = _serializer.int8(obj.stick_left_v, buffer, bufferOffset);
    // Serialize message field [stick_right_h]
    bufferOffset = _serializer.int8(obj.stick_right_h, buffer, bufferOffset);
    // Serialize message field [stick_right_v]
    bufferOffset = _serializer.int8(obj.stick_right_v, buffer, bufferOffset);
    // Serialize message field [swa]
    bufferOffset = _serializer.uint8(obj.swa, buffer, bufferOffset);
    // Serialize message field [swb]
    bufferOffset = _serializer.uint8(obj.swb, buffer, bufferOffset);
    // Serialize message field [swc]
    bufferOffset = _serializer.uint8(obj.swc, buffer, bufferOffset);
    // Serialize message field [swd]
    bufferOffset = _serializer.uint8(obj.swd, buffer, bufferOffset);
    // Serialize message field [var_a]
    bufferOffset = _serializer.uint8(obj.var_a, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ScoutRsStatus
    let len;
    let data = new ScoutRsStatus(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [stick_left_h]
    data.stick_left_h = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [stick_left_v]
    data.stick_left_v = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [stick_right_h]
    data.stick_right_h = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [stick_right_v]
    data.stick_right_v = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [swa]
    data.swa = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [swb]
    data.swb = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [swc]
    data.swc = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [swd]
    data.swd = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [var_a]
    data.var_a = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 9;
  }

  static datatype() {
    // Returns string type for a message object
    return 'scout_msgs/ScoutRsStatus';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e023f9f94b48268948746d41e8f52039';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    
    int8 stick_left_h
    int8 stick_left_v
    int8 stick_right_h
    int8 stick_right_v
    
    uint8 swa
    uint8 swb
    uint8 swc
    uint8 swd
    
    uint8 var_a
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ScoutRsStatus(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.stick_left_h !== undefined) {
      resolved.stick_left_h = msg.stick_left_h;
    }
    else {
      resolved.stick_left_h = 0
    }

    if (msg.stick_left_v !== undefined) {
      resolved.stick_left_v = msg.stick_left_v;
    }
    else {
      resolved.stick_left_v = 0
    }

    if (msg.stick_right_h !== undefined) {
      resolved.stick_right_h = msg.stick_right_h;
    }
    else {
      resolved.stick_right_h = 0
    }

    if (msg.stick_right_v !== undefined) {
      resolved.stick_right_v = msg.stick_right_v;
    }
    else {
      resolved.stick_right_v = 0
    }

    if (msg.swa !== undefined) {
      resolved.swa = msg.swa;
    }
    else {
      resolved.swa = 0
    }

    if (msg.swb !== undefined) {
      resolved.swb = msg.swb;
    }
    else {
      resolved.swb = 0
    }

    if (msg.swc !== undefined) {
      resolved.swc = msg.swc;
    }
    else {
      resolved.swc = 0
    }

    if (msg.swd !== undefined) {
      resolved.swd = msg.swd;
    }
    else {
      resolved.swd = 0
    }

    if (msg.var_a !== undefined) {
      resolved.var_a = msg.var_a;
    }
    else {
      resolved.var_a = 0
    }

    return resolved;
    }
};

module.exports = ScoutRsStatus;
