// Auto-generated. Do not edit!

// (in-package keyboard_reader.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Key {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.key_code = null;
      this.key_name = null;
      this.key_pressed = null;
    }
    else {
      if (initObj.hasOwnProperty('key_code')) {
        this.key_code = initObj.key_code
      }
      else {
        this.key_code = 0;
      }
      if (initObj.hasOwnProperty('key_name')) {
        this.key_name = initObj.key_name
      }
      else {
        this.key_name = '';
      }
      if (initObj.hasOwnProperty('key_pressed')) {
        this.key_pressed = initObj.key_pressed
      }
      else {
        this.key_pressed = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Key
    // Serialize message field [key_code]
    bufferOffset = _serializer.uint16(obj.key_code, buffer, bufferOffset);
    // Serialize message field [key_name]
    bufferOffset = _serializer.string(obj.key_name, buffer, bufferOffset);
    // Serialize message field [key_pressed]
    bufferOffset = _serializer.bool(obj.key_pressed, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Key
    let len;
    let data = new Key(null);
    // Deserialize message field [key_code]
    data.key_code = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [key_name]
    data.key_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [key_pressed]
    data.key_pressed = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.key_name.length;
    return length + 7;
  }

  static datatype() {
    // Returns string type for a message object
    return 'keyboard_reader/Key';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9709d7232efeba3860fec95e77ac1ae6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Key code as defined in linux/inupt.h
    uint16 key_code
    
    # Key name string as defined in evtest, see http://elinux.org/images/9/93/Evtest.c
    string key_name
    
    # 'True' if key was pressed, 'False' otherwise
    bool key_pressed
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Key(null);
    if (msg.key_code !== undefined) {
      resolved.key_code = msg.key_code;
    }
    else {
      resolved.key_code = 0
    }

    if (msg.key_name !== undefined) {
      resolved.key_name = msg.key_name;
    }
    else {
      resolved.key_name = ''
    }

    if (msg.key_pressed !== undefined) {
      resolved.key_pressed = msg.key_pressed;
    }
    else {
      resolved.key_pressed = false
    }

    return resolved;
    }
};

module.exports = Key;
