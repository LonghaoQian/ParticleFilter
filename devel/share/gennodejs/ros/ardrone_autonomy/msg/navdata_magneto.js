// Auto-generated. Do not edit!

// (in-package ardrone_autonomy.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let vector31 = require('./vector31.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class navdata_magneto {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.drone_time = null;
      this.tag = null;
      this.size = null;
      this.mx = null;
      this.my = null;
      this.mz = null;
      this.magneto_raw = null;
      this.magneto_rectified = null;
      this.magneto_offset = null;
      this.heading_unwrapped = null;
      this.heading_gyro_unwrapped = null;
      this.heading_fusion_unwrapped = null;
      this.magneto_calibration_ok = null;
      this.magneto_state = null;
      this.magneto_radius = null;
      this.error_mean = null;
      this.error_var = null;
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
      if (initObj.hasOwnProperty('mx')) {
        this.mx = initObj.mx
      }
      else {
        this.mx = 0;
      }
      if (initObj.hasOwnProperty('my')) {
        this.my = initObj.my
      }
      else {
        this.my = 0;
      }
      if (initObj.hasOwnProperty('mz')) {
        this.mz = initObj.mz
      }
      else {
        this.mz = 0;
      }
      if (initObj.hasOwnProperty('magneto_raw')) {
        this.magneto_raw = initObj.magneto_raw
      }
      else {
        this.magneto_raw = new vector31();
      }
      if (initObj.hasOwnProperty('magneto_rectified')) {
        this.magneto_rectified = initObj.magneto_rectified
      }
      else {
        this.magneto_rectified = new vector31();
      }
      if (initObj.hasOwnProperty('magneto_offset')) {
        this.magneto_offset = initObj.magneto_offset
      }
      else {
        this.magneto_offset = new vector31();
      }
      if (initObj.hasOwnProperty('heading_unwrapped')) {
        this.heading_unwrapped = initObj.heading_unwrapped
      }
      else {
        this.heading_unwrapped = 0.0;
      }
      if (initObj.hasOwnProperty('heading_gyro_unwrapped')) {
        this.heading_gyro_unwrapped = initObj.heading_gyro_unwrapped
      }
      else {
        this.heading_gyro_unwrapped = 0.0;
      }
      if (initObj.hasOwnProperty('heading_fusion_unwrapped')) {
        this.heading_fusion_unwrapped = initObj.heading_fusion_unwrapped
      }
      else {
        this.heading_fusion_unwrapped = 0.0;
      }
      if (initObj.hasOwnProperty('magneto_calibration_ok')) {
        this.magneto_calibration_ok = initObj.magneto_calibration_ok
      }
      else {
        this.magneto_calibration_ok = 0;
      }
      if (initObj.hasOwnProperty('magneto_state')) {
        this.magneto_state = initObj.magneto_state
      }
      else {
        this.magneto_state = 0;
      }
      if (initObj.hasOwnProperty('magneto_radius')) {
        this.magneto_radius = initObj.magneto_radius
      }
      else {
        this.magneto_radius = 0.0;
      }
      if (initObj.hasOwnProperty('error_mean')) {
        this.error_mean = initObj.error_mean
      }
      else {
        this.error_mean = 0.0;
      }
      if (initObj.hasOwnProperty('error_var')) {
        this.error_var = initObj.error_var
      }
      else {
        this.error_var = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type navdata_magneto
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [drone_time]
    bufferOffset = _serializer.float64(obj.drone_time, buffer, bufferOffset);
    // Serialize message field [tag]
    bufferOffset = _serializer.uint16(obj.tag, buffer, bufferOffset);
    // Serialize message field [size]
    bufferOffset = _serializer.uint16(obj.size, buffer, bufferOffset);
    // Serialize message field [mx]
    bufferOffset = _serializer.int16(obj.mx, buffer, bufferOffset);
    // Serialize message field [my]
    bufferOffset = _serializer.int16(obj.my, buffer, bufferOffset);
    // Serialize message field [mz]
    bufferOffset = _serializer.int16(obj.mz, buffer, bufferOffset);
    // Serialize message field [magneto_raw]
    bufferOffset = vector31.serialize(obj.magneto_raw, buffer, bufferOffset);
    // Serialize message field [magneto_rectified]
    bufferOffset = vector31.serialize(obj.magneto_rectified, buffer, bufferOffset);
    // Serialize message field [magneto_offset]
    bufferOffset = vector31.serialize(obj.magneto_offset, buffer, bufferOffset);
    // Serialize message field [heading_unwrapped]
    bufferOffset = _serializer.float32(obj.heading_unwrapped, buffer, bufferOffset);
    // Serialize message field [heading_gyro_unwrapped]
    bufferOffset = _serializer.float32(obj.heading_gyro_unwrapped, buffer, bufferOffset);
    // Serialize message field [heading_fusion_unwrapped]
    bufferOffset = _serializer.float32(obj.heading_fusion_unwrapped, buffer, bufferOffset);
    // Serialize message field [magneto_calibration_ok]
    bufferOffset = _serializer.uint8(obj.magneto_calibration_ok, buffer, bufferOffset);
    // Serialize message field [magneto_state]
    bufferOffset = _serializer.uint32(obj.magneto_state, buffer, bufferOffset);
    // Serialize message field [magneto_radius]
    bufferOffset = _serializer.float32(obj.magneto_radius, buffer, bufferOffset);
    // Serialize message field [error_mean]
    bufferOffset = _serializer.float32(obj.error_mean, buffer, bufferOffset);
    // Serialize message field [error_var]
    bufferOffset = _serializer.float32(obj.error_var, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type navdata_magneto
    let len;
    let data = new navdata_magneto(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [drone_time]
    data.drone_time = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [tag]
    data.tag = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [size]
    data.size = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [mx]
    data.mx = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [my]
    data.my = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [mz]
    data.mz = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [magneto_raw]
    data.magneto_raw = vector31.deserialize(buffer, bufferOffset);
    // Deserialize message field [magneto_rectified]
    data.magneto_rectified = vector31.deserialize(buffer, bufferOffset);
    // Deserialize message field [magneto_offset]
    data.magneto_offset = vector31.deserialize(buffer, bufferOffset);
    // Deserialize message field [heading_unwrapped]
    data.heading_unwrapped = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [heading_gyro_unwrapped]
    data.heading_gyro_unwrapped = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [heading_fusion_unwrapped]
    data.heading_fusion_unwrapped = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [magneto_calibration_ok]
    data.magneto_calibration_ok = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [magneto_state]
    data.magneto_state = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [magneto_radius]
    data.magneto_radius = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [error_mean]
    data.error_mean = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [error_var]
    data.error_var = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 83;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ardrone_autonomy/navdata_magneto';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c256b1c1d1ff0cb12a13c36720b3e224';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    float64 drone_time
    uint16 tag
    uint16 size
    int16 mx
    int16 my
    int16 mz
    vector31 magneto_raw
    vector31 magneto_rectified
    vector31 magneto_offset
    float32 heading_unwrapped
    float32 heading_gyro_unwrapped
    float32 heading_fusion_unwrapped
    uint8 magneto_calibration_ok
    uint32 magneto_state
    float32 magneto_radius
    float32 error_mean
    float32 error_var
    
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
    
    ================================================================================
    MSG: ardrone_autonomy/vector31
    float32 x
    float32 y
    float32 z
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new navdata_magneto(null);
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

    if (msg.mx !== undefined) {
      resolved.mx = msg.mx;
    }
    else {
      resolved.mx = 0
    }

    if (msg.my !== undefined) {
      resolved.my = msg.my;
    }
    else {
      resolved.my = 0
    }

    if (msg.mz !== undefined) {
      resolved.mz = msg.mz;
    }
    else {
      resolved.mz = 0
    }

    if (msg.magneto_raw !== undefined) {
      resolved.magneto_raw = vector31.Resolve(msg.magneto_raw)
    }
    else {
      resolved.magneto_raw = new vector31()
    }

    if (msg.magneto_rectified !== undefined) {
      resolved.magneto_rectified = vector31.Resolve(msg.magneto_rectified)
    }
    else {
      resolved.magneto_rectified = new vector31()
    }

    if (msg.magneto_offset !== undefined) {
      resolved.magneto_offset = vector31.Resolve(msg.magneto_offset)
    }
    else {
      resolved.magneto_offset = new vector31()
    }

    if (msg.heading_unwrapped !== undefined) {
      resolved.heading_unwrapped = msg.heading_unwrapped;
    }
    else {
      resolved.heading_unwrapped = 0.0
    }

    if (msg.heading_gyro_unwrapped !== undefined) {
      resolved.heading_gyro_unwrapped = msg.heading_gyro_unwrapped;
    }
    else {
      resolved.heading_gyro_unwrapped = 0.0
    }

    if (msg.heading_fusion_unwrapped !== undefined) {
      resolved.heading_fusion_unwrapped = msg.heading_fusion_unwrapped;
    }
    else {
      resolved.heading_fusion_unwrapped = 0.0
    }

    if (msg.magneto_calibration_ok !== undefined) {
      resolved.magneto_calibration_ok = msg.magneto_calibration_ok;
    }
    else {
      resolved.magneto_calibration_ok = 0
    }

    if (msg.magneto_state !== undefined) {
      resolved.magneto_state = msg.magneto_state;
    }
    else {
      resolved.magneto_state = 0
    }

    if (msg.magneto_radius !== undefined) {
      resolved.magneto_radius = msg.magneto_radius;
    }
    else {
      resolved.magneto_radius = 0.0
    }

    if (msg.error_mean !== undefined) {
      resolved.error_mean = msg.error_mean;
    }
    else {
      resolved.error_mean = 0.0
    }

    if (msg.error_var !== undefined) {
      resolved.error_var = msg.error_var;
    }
    else {
      resolved.error_var = 0.0
    }

    return resolved;
    }
};

module.exports = navdata_magneto;
