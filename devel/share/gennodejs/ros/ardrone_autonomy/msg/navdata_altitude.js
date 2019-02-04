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
let vector21 = require('./vector21.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class navdata_altitude {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.drone_time = null;
      this.tag = null;
      this.size = null;
      this.altitude_vision = null;
      this.altitude_vz = null;
      this.altitude_ref = null;
      this.altitude_raw = null;
      this.obs_accZ = null;
      this.obs_alt = null;
      this.obs_x = null;
      this.obs_state = null;
      this.est_vb = null;
      this.est_state = null;
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
      if (initObj.hasOwnProperty('altitude_vision')) {
        this.altitude_vision = initObj.altitude_vision
      }
      else {
        this.altitude_vision = 0;
      }
      if (initObj.hasOwnProperty('altitude_vz')) {
        this.altitude_vz = initObj.altitude_vz
      }
      else {
        this.altitude_vz = 0.0;
      }
      if (initObj.hasOwnProperty('altitude_ref')) {
        this.altitude_ref = initObj.altitude_ref
      }
      else {
        this.altitude_ref = 0;
      }
      if (initObj.hasOwnProperty('altitude_raw')) {
        this.altitude_raw = initObj.altitude_raw
      }
      else {
        this.altitude_raw = 0;
      }
      if (initObj.hasOwnProperty('obs_accZ')) {
        this.obs_accZ = initObj.obs_accZ
      }
      else {
        this.obs_accZ = 0.0;
      }
      if (initObj.hasOwnProperty('obs_alt')) {
        this.obs_alt = initObj.obs_alt
      }
      else {
        this.obs_alt = 0.0;
      }
      if (initObj.hasOwnProperty('obs_x')) {
        this.obs_x = initObj.obs_x
      }
      else {
        this.obs_x = new vector31();
      }
      if (initObj.hasOwnProperty('obs_state')) {
        this.obs_state = initObj.obs_state
      }
      else {
        this.obs_state = 0;
      }
      if (initObj.hasOwnProperty('est_vb')) {
        this.est_vb = initObj.est_vb
      }
      else {
        this.est_vb = new vector21();
      }
      if (initObj.hasOwnProperty('est_state')) {
        this.est_state = initObj.est_state
      }
      else {
        this.est_state = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type navdata_altitude
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [drone_time]
    bufferOffset = _serializer.float64(obj.drone_time, buffer, bufferOffset);
    // Serialize message field [tag]
    bufferOffset = _serializer.uint16(obj.tag, buffer, bufferOffset);
    // Serialize message field [size]
    bufferOffset = _serializer.uint16(obj.size, buffer, bufferOffset);
    // Serialize message field [altitude_vision]
    bufferOffset = _serializer.int32(obj.altitude_vision, buffer, bufferOffset);
    // Serialize message field [altitude_vz]
    bufferOffset = _serializer.float32(obj.altitude_vz, buffer, bufferOffset);
    // Serialize message field [altitude_ref]
    bufferOffset = _serializer.int32(obj.altitude_ref, buffer, bufferOffset);
    // Serialize message field [altitude_raw]
    bufferOffset = _serializer.int32(obj.altitude_raw, buffer, bufferOffset);
    // Serialize message field [obs_accZ]
    bufferOffset = _serializer.float32(obj.obs_accZ, buffer, bufferOffset);
    // Serialize message field [obs_alt]
    bufferOffset = _serializer.float32(obj.obs_alt, buffer, bufferOffset);
    // Serialize message field [obs_x]
    bufferOffset = vector31.serialize(obj.obs_x, buffer, bufferOffset);
    // Serialize message field [obs_state]
    bufferOffset = _serializer.uint32(obj.obs_state, buffer, bufferOffset);
    // Serialize message field [est_vb]
    bufferOffset = vector21.serialize(obj.est_vb, buffer, bufferOffset);
    // Serialize message field [est_state]
    bufferOffset = _serializer.uint32(obj.est_state, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type navdata_altitude
    let len;
    let data = new navdata_altitude(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [drone_time]
    data.drone_time = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [tag]
    data.tag = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [size]
    data.size = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [altitude_vision]
    data.altitude_vision = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [altitude_vz]
    data.altitude_vz = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [altitude_ref]
    data.altitude_ref = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [altitude_raw]
    data.altitude_raw = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [obs_accZ]
    data.obs_accZ = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [obs_alt]
    data.obs_alt = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [obs_x]
    data.obs_x = vector31.deserialize(buffer, bufferOffset);
    // Deserialize message field [obs_state]
    data.obs_state = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [est_vb]
    data.est_vb = vector21.deserialize(buffer, bufferOffset);
    // Deserialize message field [est_state]
    data.est_state = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 64;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ardrone_autonomy/navdata_altitude';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f0fd1fd20697e6083c6bc3a308a260dc';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    float64 drone_time
    uint16 tag
    uint16 size
    int32 altitude_vision
    float32 altitude_vz
    int32 altitude_ref
    int32 altitude_raw
    float32 obs_accZ
    float32 obs_alt
    vector31 obs_x
    uint32 obs_state
    vector21 est_vb
    uint32 est_state
    
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
    ================================================================================
    MSG: ardrone_autonomy/vector21
    float32 x
    float32 y
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new navdata_altitude(null);
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

    if (msg.altitude_vision !== undefined) {
      resolved.altitude_vision = msg.altitude_vision;
    }
    else {
      resolved.altitude_vision = 0
    }

    if (msg.altitude_vz !== undefined) {
      resolved.altitude_vz = msg.altitude_vz;
    }
    else {
      resolved.altitude_vz = 0.0
    }

    if (msg.altitude_ref !== undefined) {
      resolved.altitude_ref = msg.altitude_ref;
    }
    else {
      resolved.altitude_ref = 0
    }

    if (msg.altitude_raw !== undefined) {
      resolved.altitude_raw = msg.altitude_raw;
    }
    else {
      resolved.altitude_raw = 0
    }

    if (msg.obs_accZ !== undefined) {
      resolved.obs_accZ = msg.obs_accZ;
    }
    else {
      resolved.obs_accZ = 0.0
    }

    if (msg.obs_alt !== undefined) {
      resolved.obs_alt = msg.obs_alt;
    }
    else {
      resolved.obs_alt = 0.0
    }

    if (msg.obs_x !== undefined) {
      resolved.obs_x = vector31.Resolve(msg.obs_x)
    }
    else {
      resolved.obs_x = new vector31()
    }

    if (msg.obs_state !== undefined) {
      resolved.obs_state = msg.obs_state;
    }
    else {
      resolved.obs_state = 0
    }

    if (msg.est_vb !== undefined) {
      resolved.est_vb = vector21.Resolve(msg.est_vb)
    }
    else {
      resolved.est_vb = new vector21()
    }

    if (msg.est_state !== undefined) {
      resolved.est_state = msg.est_state;
    }
    else {
      resolved.est_state = 0
    }

    return resolved;
    }
};

module.exports = navdata_altitude;
