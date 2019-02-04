// Auto-generated. Do not edit!

// (in-package ardrone_autonomy.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class navdata_pressure_raw {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.drone_time = null;
      this.tag = null;
      this.size = null;
      this.up = null;
      this.ut = null;
      this.Temperature_meas = null;
      this.Pression_meas = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('drone_time')) {
        this.drone_time = initObj.drone_time
      }
      else {
        this.drone_time = 0.0;
      }
      if (initObj.hasOwnProperty('tag')) {
        this.tag = initObj.tag
      }
      else {
        this.tag = 0;
      }
      if (initObj.hasOwnProperty('size')) {
        this.size = initObj.size
      }
      else {
        this.size = 0;
      }
      if (initObj.hasOwnProperty('up')) {
        this.up = initObj.up
      }
      else {
        this.up = 0;
      }
      if (initObj.hasOwnProperty('ut')) {
        this.ut = initObj.ut
      }
      else {
        this.ut = 0;
      }
      if (initObj.hasOwnProperty('Temperature_meas')) {
        this.Temperature_meas = initObj.Temperature_meas
      }
      else {
        this.Temperature_meas = 0;
      }
      if (initObj.hasOwnProperty('Pression_meas')) {
        this.Pression_meas = initObj.Pression_meas
      }
      else {
        this.Pression_meas = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type navdata_pressure_raw
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [drone_time]
    bufferOffset = _serializer.float64(obj.drone_time, buffer, bufferOffset);
    // Serialize message field [tag]
    bufferOffset = _serializer.uint16(obj.tag, buffer, bufferOffset);
    // Serialize message field [size]
    bufferOffset = _serializer.uint16(obj.size, buffer, bufferOffset);
    // Serialize message field [up]
    bufferOffset = _serializer.int32(obj.up, buffer, bufferOffset);
    // Serialize message field [ut]
    bufferOffset = _serializer.int16(obj.ut, buffer, bufferOffset);
    // Serialize message field [Temperature_meas]
    bufferOffset = _serializer.int32(obj.Temperature_meas, buffer, bufferOffset);
    // Serialize message field [Pression_meas]
    bufferOffset = _serializer.int32(obj.Pression_meas, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type navdata_pressure_raw
    let len;
    let data = new navdata_pressure_raw(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [drone_time]
    data.drone_time = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [tag]
    data.tag = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [size]
    data.size = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [up]
    data.up = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [ut]
    data.ut = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [Temperature_meas]
    data.Temperature_meas = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [Pression_meas]
    data.Pression_meas = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 26;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ardrone_autonomy/navdata_pressure_raw';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'bee798af5d1297cfcb39217d797b8d90';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    float64 drone_time
    uint16 tag
    uint16 size
    int32 up
    int16 ut
    int32 Temperature_meas
    int32 Pression_meas
    
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
    # 0: no frame
    # 1: global frame
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new navdata_pressure_raw(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.drone_time !== undefined) {
      resolved.drone_time = msg.drone_time;
    }
    else {
      resolved.drone_time = 0.0
    }

    if (msg.tag !== undefined) {
      resolved.tag = msg.tag;
    }
    else {
      resolved.tag = 0
    }

    if (msg.size !== undefined) {
      resolved.size = msg.size;
    }
    else {
      resolved.size = 0
    }

    if (msg.up !== undefined) {
      resolved.up = msg.up;
    }
    else {
      resolved.up = 0
    }

    if (msg.ut !== undefined) {
      resolved.ut = msg.ut;
    }
    else {
      resolved.ut = 0
    }

    if (msg.Temperature_meas !== undefined) {
      resolved.Temperature_meas = msg.Temperature_meas;
    }
    else {
      resolved.Temperature_meas = 0
    }

    if (msg.Pression_meas !== undefined) {
      resolved.Pression_meas = msg.Pression_meas;
    }
    else {
      resolved.Pression_meas = 0
    }

    return resolved;
    }
};

module.exports = navdata_pressure_raw;
