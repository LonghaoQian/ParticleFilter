// Auto-generated. Do not edit!

// (in-package ardrone_autonomy.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class matrix33 {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.m11 = null;
      this.m12 = null;
      this.m13 = null;
      this.m21 = null;
      this.m22 = null;
      this.m23 = null;
      this.m31 = null;
      this.m32 = null;
      this.m33 = null;
    }
    else {
      if (initObj.hasOwnProperty('m11')) {
        this.m11 = initObj.m11
      }
      else {
        this.m11 = 0.0;
      }
      if (initObj.hasOwnProperty('m12')) {
        this.m12 = initObj.m12
      }
      else {
        this.m12 = 0.0;
      }
      if (initObj.hasOwnProperty('m13')) {
        this.m13 = initObj.m13
      }
      else {
        this.m13 = 0.0;
      }
      if (initObj.hasOwnProperty('m21')) {
        this.m21 = initObj.m21
      }
      else {
        this.m21 = 0.0;
      }
      if (initObj.hasOwnProperty('m22')) {
        this.m22 = initObj.m22
      }
      else {
        this.m22 = 0.0;
      }
      if (initObj.hasOwnProperty('m23')) {
        this.m23 = initObj.m23
      }
      else {
        this.m23 = 0.0;
      }
      if (initObj.hasOwnProperty('m31')) {
        this.m31 = initObj.m31
      }
      else {
        this.m31 = 0.0;
      }
      if (initObj.hasOwnProperty('m32')) {
        this.m32 = initObj.m32
      }
      else {
        this.m32 = 0.0;
      }
      if (initObj.hasOwnProperty('m33')) {
        this.m33 = initObj.m33
      }
      else {
        this.m33 = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type matrix33
    // Serialize message field [m11]
    bufferOffset = _serializer.float32(obj.m11, buffer, bufferOffset);
    // Serialize message field [m12]
    bufferOffset = _serializer.float32(obj.m12, buffer, bufferOffset);
    // Serialize message field [m13]
    bufferOffset = _serializer.float32(obj.m13, buffer, bufferOffset);
    // Serialize message field [m21]
    bufferOffset = _serializer.float32(obj.m21, buffer, bufferOffset);
    // Serialize message field [m22]
    bufferOffset = _serializer.float32(obj.m22, buffer, bufferOffset);
    // Serialize message field [m23]
    bufferOffset = _serializer.float32(obj.m23, buffer, bufferOffset);
    // Serialize message field [m31]
    bufferOffset = _serializer.float32(obj.m31, buffer, bufferOffset);
    // Serialize message field [m32]
    bufferOffset = _serializer.float32(obj.m32, buffer, bufferOffset);
    // Serialize message field [m33]
    bufferOffset = _serializer.float32(obj.m33, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type matrix33
    let len;
    let data = new matrix33(null);
    // Deserialize message field [m11]
    data.m11 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [m12]
    data.m12 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [m13]
    data.m13 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [m21]
    data.m21 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [m22]
    data.m22 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [m23]
    data.m23 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [m31]
    data.m31 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [m32]
    data.m32 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [m33]
    data.m33 = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 36;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ardrone_autonomy/matrix33';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd6abfd8481fcb7c359a8d5d9388fa257';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 m11
    float32 m12
    float32 m13
    float32 m21
    float32 m22
    float32 m23
    float32 m31
    float32 m32
    float32 m33
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new matrix33(null);
    if (msg.m11 !== undefined) {
      resolved.m11 = msg.m11;
    }
    else {
      resolved.m11 = 0.0
    }

    if (msg.m12 !== undefined) {
      resolved.m12 = msg.m12;
    }
    else {
      resolved.m12 = 0.0
    }

    if (msg.m13 !== undefined) {
      resolved.m13 = msg.m13;
    }
    else {
      resolved.m13 = 0.0
    }

    if (msg.m21 !== undefined) {
      resolved.m21 = msg.m21;
    }
    else {
      resolved.m21 = 0.0
    }

    if (msg.m22 !== undefined) {
      resolved.m22 = msg.m22;
    }
    else {
      resolved.m22 = 0.0
    }

    if (msg.m23 !== undefined) {
      resolved.m23 = msg.m23;
    }
    else {
      resolved.m23 = 0.0
    }

    if (msg.m31 !== undefined) {
      resolved.m31 = msg.m31;
    }
    else {
      resolved.m31 = 0.0
    }

    if (msg.m32 !== undefined) {
      resolved.m32 = msg.m32;
    }
    else {
      resolved.m32 = 0.0
    }

    if (msg.m33 !== undefined) {
      resolved.m33 = msg.m33;
    }
    else {
      resolved.m33 = 0.0
    }

    return resolved;
    }
};

module.exports = matrix33;
