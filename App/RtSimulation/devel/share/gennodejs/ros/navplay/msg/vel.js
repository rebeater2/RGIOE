// Auto-generated. Do not edit!

// (in-package navplay.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class vel {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.gpst = null;
      this.forward = null;
      this.angular = null;
    }
    else {
      if (initObj.hasOwnProperty('gpst')) {
        this.gpst = initObj.gpst
      }
      else {
        this.gpst = 0.0;
      }
      if (initObj.hasOwnProperty('forward')) {
        this.forward = initObj.forward
      }
      else {
        this.forward = 0.0;
      }
      if (initObj.hasOwnProperty('angular')) {
        this.angular = initObj.angular
      }
      else {
        this.angular = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type vel
    // Serialize message field [gpst]
    bufferOffset = _serializer.float64(obj.gpst, buffer, bufferOffset);
    // Serialize message field [forward]
    bufferOffset = _serializer.float32(obj.forward, buffer, bufferOffset);
    // Serialize message field [angular]
    bufferOffset = _serializer.float32(obj.angular, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type vel
    let len;
    let data = new vel(null);
    // Deserialize message field [gpst]
    data.gpst = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [forward]
    data.forward = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [angular]
    data.angular = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'navplay/vel';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7e11ee5e5f9a364893511f8eb7b4a576';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 gpst
    float32 forward
    float32 angular
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new vel(null);
    if (msg.gpst !== undefined) {
      resolved.gpst = msg.gpst;
    }
    else {
      resolved.gpst = 0.0
    }

    if (msg.forward !== undefined) {
      resolved.forward = msg.forward;
    }
    else {
      resolved.forward = 0.0
    }

    if (msg.angular !== undefined) {
      resolved.angular = msg.angular;
    }
    else {
      resolved.angular = 0.0
    }

    return resolved;
    }
};

module.exports = vel;
