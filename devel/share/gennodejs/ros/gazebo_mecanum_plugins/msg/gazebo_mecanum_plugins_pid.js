// Auto-generated. Do not edit!

// (in-package gazebo_mecanum_plugins.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class gazebo_mecanum_plugins_pid {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.LR_P = null;
      this.LR_I = null;
      this.LR_D = null;
      this.LF_P = null;
      this.LF_I = null;
      this.LF_D = null;
      this.RF_P = null;
      this.RF_I = null;
      this.RF_D = null;
      this.RR_P = null;
      this.RR_I = null;
      this.RR_D = null;
    }
    else {
      if (initObj.hasOwnProperty('LR_P')) {
        this.LR_P = initObj.LR_P
      }
      else {
        this.LR_P = 0.0;
      }
      if (initObj.hasOwnProperty('LR_I')) {
        this.LR_I = initObj.LR_I
      }
      else {
        this.LR_I = 0.0;
      }
      if (initObj.hasOwnProperty('LR_D')) {
        this.LR_D = initObj.LR_D
      }
      else {
        this.LR_D = 0.0;
      }
      if (initObj.hasOwnProperty('LF_P')) {
        this.LF_P = initObj.LF_P
      }
      else {
        this.LF_P = 0.0;
      }
      if (initObj.hasOwnProperty('LF_I')) {
        this.LF_I = initObj.LF_I
      }
      else {
        this.LF_I = 0.0;
      }
      if (initObj.hasOwnProperty('LF_D')) {
        this.LF_D = initObj.LF_D
      }
      else {
        this.LF_D = 0.0;
      }
      if (initObj.hasOwnProperty('RF_P')) {
        this.RF_P = initObj.RF_P
      }
      else {
        this.RF_P = 0.0;
      }
      if (initObj.hasOwnProperty('RF_I')) {
        this.RF_I = initObj.RF_I
      }
      else {
        this.RF_I = 0.0;
      }
      if (initObj.hasOwnProperty('RF_D')) {
        this.RF_D = initObj.RF_D
      }
      else {
        this.RF_D = 0.0;
      }
      if (initObj.hasOwnProperty('RR_P')) {
        this.RR_P = initObj.RR_P
      }
      else {
        this.RR_P = 0.0;
      }
      if (initObj.hasOwnProperty('RR_I')) {
        this.RR_I = initObj.RR_I
      }
      else {
        this.RR_I = 0.0;
      }
      if (initObj.hasOwnProperty('RR_D')) {
        this.RR_D = initObj.RR_D
      }
      else {
        this.RR_D = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type gazebo_mecanum_plugins_pid
    // Serialize message field [LR_P]
    bufferOffset = _serializer.float64(obj.LR_P, buffer, bufferOffset);
    // Serialize message field [LR_I]
    bufferOffset = _serializer.float64(obj.LR_I, buffer, bufferOffset);
    // Serialize message field [LR_D]
    bufferOffset = _serializer.float64(obj.LR_D, buffer, bufferOffset);
    // Serialize message field [LF_P]
    bufferOffset = _serializer.float64(obj.LF_P, buffer, bufferOffset);
    // Serialize message field [LF_I]
    bufferOffset = _serializer.float64(obj.LF_I, buffer, bufferOffset);
    // Serialize message field [LF_D]
    bufferOffset = _serializer.float64(obj.LF_D, buffer, bufferOffset);
    // Serialize message field [RF_P]
    bufferOffset = _serializer.float64(obj.RF_P, buffer, bufferOffset);
    // Serialize message field [RF_I]
    bufferOffset = _serializer.float64(obj.RF_I, buffer, bufferOffset);
    // Serialize message field [RF_D]
    bufferOffset = _serializer.float64(obj.RF_D, buffer, bufferOffset);
    // Serialize message field [RR_P]
    bufferOffset = _serializer.float64(obj.RR_P, buffer, bufferOffset);
    // Serialize message field [RR_I]
    bufferOffset = _serializer.float64(obj.RR_I, buffer, bufferOffset);
    // Serialize message field [RR_D]
    bufferOffset = _serializer.float64(obj.RR_D, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type gazebo_mecanum_plugins_pid
    let len;
    let data = new gazebo_mecanum_plugins_pid(null);
    // Deserialize message field [LR_P]
    data.LR_P = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [LR_I]
    data.LR_I = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [LR_D]
    data.LR_D = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [LF_P]
    data.LF_P = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [LF_I]
    data.LF_I = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [LF_D]
    data.LF_D = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [RF_P]
    data.RF_P = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [RF_I]
    data.RF_I = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [RF_D]
    data.RF_D = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [RR_P]
    data.RR_P = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [RR_I]
    data.RR_I = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [RR_D]
    data.RR_D = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 96;
  }

  static datatype() {
    // Returns string type for a message object
    return 'gazebo_mecanum_plugins/gazebo_mecanum_plugins_pid';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9eb89efea79b06cdde55828e8174754b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 LR_P
    float64 LR_I
    float64 LR_D
    float64 LF_P
    float64 LF_I
    float64 LF_D
    float64 RF_P
    float64 RF_I
    float64 RF_D
    float64 RR_P
    float64 RR_I
    float64 RR_D
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new gazebo_mecanum_plugins_pid(null);
    if (msg.LR_P !== undefined) {
      resolved.LR_P = msg.LR_P;
    }
    else {
      resolved.LR_P = 0.0
    }

    if (msg.LR_I !== undefined) {
      resolved.LR_I = msg.LR_I;
    }
    else {
      resolved.LR_I = 0.0
    }

    if (msg.LR_D !== undefined) {
      resolved.LR_D = msg.LR_D;
    }
    else {
      resolved.LR_D = 0.0
    }

    if (msg.LF_P !== undefined) {
      resolved.LF_P = msg.LF_P;
    }
    else {
      resolved.LF_P = 0.0
    }

    if (msg.LF_I !== undefined) {
      resolved.LF_I = msg.LF_I;
    }
    else {
      resolved.LF_I = 0.0
    }

    if (msg.LF_D !== undefined) {
      resolved.LF_D = msg.LF_D;
    }
    else {
      resolved.LF_D = 0.0
    }

    if (msg.RF_P !== undefined) {
      resolved.RF_P = msg.RF_P;
    }
    else {
      resolved.RF_P = 0.0
    }

    if (msg.RF_I !== undefined) {
      resolved.RF_I = msg.RF_I;
    }
    else {
      resolved.RF_I = 0.0
    }

    if (msg.RF_D !== undefined) {
      resolved.RF_D = msg.RF_D;
    }
    else {
      resolved.RF_D = 0.0
    }

    if (msg.RR_P !== undefined) {
      resolved.RR_P = msg.RR_P;
    }
    else {
      resolved.RR_P = 0.0
    }

    if (msg.RR_I !== undefined) {
      resolved.RR_I = msg.RR_I;
    }
    else {
      resolved.RR_I = 0.0
    }

    if (msg.RR_D !== undefined) {
      resolved.RR_D = msg.RR_D;
    }
    else {
      resolved.RR_D = 0.0
    }

    return resolved;
    }
};

module.exports = gazebo_mecanum_plugins_pid;
