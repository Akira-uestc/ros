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

class gazebo_mecanum_plugins_vel {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.LEFT_REAR_vel = null;
      this.LEFT_FRONT_vel = null;
      this.RIGHT_FRONT_vel = null;
      this.RIGHT_REAR_vel = null;
    }
    else {
      if (initObj.hasOwnProperty('LEFT_REAR_vel')) {
        this.LEFT_REAR_vel = initObj.LEFT_REAR_vel
      }
      else {
        this.LEFT_REAR_vel = 0.0;
      }
      if (initObj.hasOwnProperty('LEFT_FRONT_vel')) {
        this.LEFT_FRONT_vel = initObj.LEFT_FRONT_vel
      }
      else {
        this.LEFT_FRONT_vel = 0.0;
      }
      if (initObj.hasOwnProperty('RIGHT_FRONT_vel')) {
        this.RIGHT_FRONT_vel = initObj.RIGHT_FRONT_vel
      }
      else {
        this.RIGHT_FRONT_vel = 0.0;
      }
      if (initObj.hasOwnProperty('RIGHT_REAR_vel')) {
        this.RIGHT_REAR_vel = initObj.RIGHT_REAR_vel
      }
      else {
        this.RIGHT_REAR_vel = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type gazebo_mecanum_plugins_vel
    // Serialize message field [LEFT_REAR_vel]
    bufferOffset = _serializer.float64(obj.LEFT_REAR_vel, buffer, bufferOffset);
    // Serialize message field [LEFT_FRONT_vel]
    bufferOffset = _serializer.float64(obj.LEFT_FRONT_vel, buffer, bufferOffset);
    // Serialize message field [RIGHT_FRONT_vel]
    bufferOffset = _serializer.float64(obj.RIGHT_FRONT_vel, buffer, bufferOffset);
    // Serialize message field [RIGHT_REAR_vel]
    bufferOffset = _serializer.float64(obj.RIGHT_REAR_vel, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type gazebo_mecanum_plugins_vel
    let len;
    let data = new gazebo_mecanum_plugins_vel(null);
    // Deserialize message field [LEFT_REAR_vel]
    data.LEFT_REAR_vel = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [LEFT_FRONT_vel]
    data.LEFT_FRONT_vel = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [RIGHT_FRONT_vel]
    data.RIGHT_FRONT_vel = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [RIGHT_REAR_vel]
    data.RIGHT_REAR_vel = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 32;
  }

  static datatype() {
    // Returns string type for a message object
    return 'gazebo_mecanum_plugins/gazebo_mecanum_plugins_vel';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f9df474704d7113e588f9ced0981881c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 LEFT_REAR_vel
    float64 LEFT_FRONT_vel
    float64 RIGHT_FRONT_vel
    float64 RIGHT_REAR_vel
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new gazebo_mecanum_plugins_vel(null);
    if (msg.LEFT_REAR_vel !== undefined) {
      resolved.LEFT_REAR_vel = msg.LEFT_REAR_vel;
    }
    else {
      resolved.LEFT_REAR_vel = 0.0
    }

    if (msg.LEFT_FRONT_vel !== undefined) {
      resolved.LEFT_FRONT_vel = msg.LEFT_FRONT_vel;
    }
    else {
      resolved.LEFT_FRONT_vel = 0.0
    }

    if (msg.RIGHT_FRONT_vel !== undefined) {
      resolved.RIGHT_FRONT_vel = msg.RIGHT_FRONT_vel;
    }
    else {
      resolved.RIGHT_FRONT_vel = 0.0
    }

    if (msg.RIGHT_REAR_vel !== undefined) {
      resolved.RIGHT_REAR_vel = msg.RIGHT_REAR_vel;
    }
    else {
      resolved.RIGHT_REAR_vel = 0.0
    }

    return resolved;
    }
};

module.exports = gazebo_mecanum_plugins_vel;
