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

class gnss {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.gpst = null;
      this.lat = null;
      this.lon = null;
      this.h = null;
      this.pos_std = null;
      this.mode = null;
      this.ns = null;
    }
    else {
      if (initObj.hasOwnProperty('gpst')) {
        this.gpst = initObj.gpst
      }
      else {
        this.gpst = 0.0;
      }
      if (initObj.hasOwnProperty('lat')) {
        this.lat = initObj.lat
      }
      else {
        this.lat = 0.0;
      }
      if (initObj.hasOwnProperty('lon')) {
        this.lon = initObj.lon
      }
      else {
        this.lon = 0.0;
      }
      if (initObj.hasOwnProperty('h')) {
        this.h = initObj.h
      }
      else {
        this.h = 0.0;
      }
      if (initObj.hasOwnProperty('pos_std')) {
        this.pos_std = initObj.pos_std
      }
      else {
        this.pos_std = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('mode')) {
        this.mode = initObj.mode
      }
      else {
        this.mode = 0;
      }
      if (initObj.hasOwnProperty('ns')) {
        this.ns = initObj.ns
      }
      else {
        this.ns = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type gnss
    // Serialize message field [gpst]
    bufferOffset = _serializer.float64(obj.gpst, buffer, bufferOffset);
    // Serialize message field [lat]
    bufferOffset = _serializer.float64(obj.lat, buffer, bufferOffset);
    // Serialize message field [lon]
    bufferOffset = _serializer.float64(obj.lon, buffer, bufferOffset);
    // Serialize message field [h]
    bufferOffset = _serializer.float32(obj.h, buffer, bufferOffset);
    // Check that the constant length array field [pos_std] has the right length
    if (obj.pos_std.length !== 3) {
      throw new Error('Unable to serialize array field pos_std - length must be 3')
    }
    // Serialize message field [pos_std]
    bufferOffset = _arraySerializer.float32(obj.pos_std, buffer, bufferOffset, 3);
    // Serialize message field [mode]
    bufferOffset = _serializer.uint8(obj.mode, buffer, bufferOffset);
    // Serialize message field [ns]
    bufferOffset = _serializer.uint8(obj.ns, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type gnss
    let len;
    let data = new gnss(null);
    // Deserialize message field [gpst]
    data.gpst = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [lat]
    data.lat = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [lon]
    data.lon = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [h]
    data.h = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [pos_std]
    data.pos_std = _arrayDeserializer.float32(buffer, bufferOffset, 3)
    // Deserialize message field [mode]
    data.mode = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [ns]
    data.ns = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 42;
  }

  static datatype() {
    // Returns string type for a message object
    return 'navplay/gnss';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd8b3bc143a1901908ef5f7a6d985bd0b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 gpst
    float64 lat
    float64 lon
    float32 h
    float32[3] pos_std
    uint8 mode
    uint8 ns
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new gnss(null);
    if (msg.gpst !== undefined) {
      resolved.gpst = msg.gpst;
    }
    else {
      resolved.gpst = 0.0
    }

    if (msg.lat !== undefined) {
      resolved.lat = msg.lat;
    }
    else {
      resolved.lat = 0.0
    }

    if (msg.lon !== undefined) {
      resolved.lon = msg.lon;
    }
    else {
      resolved.lon = 0.0
    }

    if (msg.h !== undefined) {
      resolved.h = msg.h;
    }
    else {
      resolved.h = 0.0
    }

    if (msg.pos_std !== undefined) {
      resolved.pos_std = msg.pos_std;
    }
    else {
      resolved.pos_std = new Array(3).fill(0)
    }

    if (msg.mode !== undefined) {
      resolved.mode = msg.mode;
    }
    else {
      resolved.mode = 0
    }

    if (msg.ns !== undefined) {
      resolved.ns = msg.ns;
    }
    else {
      resolved.ns = 0
    }

    return resolved;
    }
};

module.exports = gnss;
