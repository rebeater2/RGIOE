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

class imu {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.gpst = null;
      this.acce = null;
      this.gyro = null;
    }
    else {
      if (initObj.hasOwnProperty('gpst')) {
        this.gpst = initObj.gpst
      }
      else {
        this.gpst = 0.0;
      }
      if (initObj.hasOwnProperty('acce')) {
        this.acce = initObj.acce
      }
      else {
        this.acce = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('gyro')) {
        this.gyro = initObj.gyro
      }
      else {
        this.gyro = new Array(3).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type imu
    // Serialize message field [gpst]
    bufferOffset = _serializer.float64(obj.gpst, buffer, bufferOffset);
    // Check that the constant length array field [acce] has the right length
    if (obj.acce.length !== 3) {
      throw new Error('Unable to serialize array field acce - length must be 3')
    }
    // Serialize message field [acce]
    bufferOffset = _arraySerializer.float64(obj.acce, buffer, bufferOffset, 3);
    // Check that the constant length array field [gyro] has the right length
    if (obj.gyro.length !== 3) {
      throw new Error('Unable to serialize array field gyro - length must be 3')
    }
    // Serialize message field [gyro]
    bufferOffset = _arraySerializer.float64(obj.gyro, buffer, bufferOffset, 3);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type imu
    let len;
    let data = new imu(null);
    // Deserialize message field [gpst]
    data.gpst = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [acce]
    data.acce = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    // Deserialize message field [gyro]
    data.gyro = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    return data;
  }

  static getMessageSize(object) {
    return 56;
  }

  static datatype() {
    // Returns string type for a message object
    return 'navplay/imu';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '08a3930f07de2d670d1b2b4f3b2dd41c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 gpst
    float64[3] acce
    float64[3] gyro
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new imu(null);
    if (msg.gpst !== undefined) {
      resolved.gpst = msg.gpst;
    }
    else {
      resolved.gpst = 0.0
    }

    if (msg.acce !== undefined) {
      resolved.acce = msg.acce;
    }
    else {
      resolved.acce = new Array(3).fill(0)
    }

    if (msg.gyro !== undefined) {
      resolved.gyro = msg.gyro;
    }
    else {
      resolved.gyro = new Array(3).fill(0)
    }

    return resolved;
    }
};

module.exports = imu;
