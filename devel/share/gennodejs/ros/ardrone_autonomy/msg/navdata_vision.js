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

class navdata_vision {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.drone_time = null;
      this.tag = null;
      this.size = null;
      this.vision_state = null;
      this.vision_misc = null;
      this.vision_phi_trim = null;
      this.vision_phi_ref_prop = null;
      this.vision_theta_trim = null;
      this.vision_theta_ref_prop = null;
      this.new_raw_picture = null;
      this.theta_capture = null;
      this.phi_capture = null;
      this.psi_capture = null;
      this.altitude_capture = null;
      this.time_capture = null;
      this.body_v = null;
      this.delta_phi = null;
      this.delta_theta = null;
      this.delta_psi = null;
      this.gold_defined = null;
      this.gold_reset = null;
      this.gold_x = null;
      this.gold_y = null;
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
      if (initObj.hasOwnProperty('vision_state')) {
        this.vision_state = initObj.vision_state
      }
      else {
        this.vision_state = 0;
      }
      if (initObj.hasOwnProperty('vision_misc')) {
        this.vision_misc = initObj.vision_misc
      }
      else {
        this.vision_misc = 0;
      }
      if (initObj.hasOwnProperty('vision_phi_trim')) {
        this.vision_phi_trim = initObj.vision_phi_trim
      }
      else {
        this.vision_phi_trim = 0.0;
      }
      if (initObj.hasOwnProperty('vision_phi_ref_prop')) {
        this.vision_phi_ref_prop = initObj.vision_phi_ref_prop
      }
      else {
        this.vision_phi_ref_prop = 0.0;
      }
      if (initObj.hasOwnProperty('vision_theta_trim')) {
        this.vision_theta_trim = initObj.vision_theta_trim
      }
      else {
        this.vision_theta_trim = 0.0;
      }
      if (initObj.hasOwnProperty('vision_theta_ref_prop')) {
        this.vision_theta_ref_prop = initObj.vision_theta_ref_prop
      }
      else {
        this.vision_theta_ref_prop = 0.0;
      }
      if (initObj.hasOwnProperty('new_raw_picture')) {
        this.new_raw_picture = initObj.new_raw_picture
      }
      else {
        this.new_raw_picture = 0;
      }
      if (initObj.hasOwnProperty('theta_capture')) {
        this.theta_capture = initObj.theta_capture
      }
      else {
        this.theta_capture = 0.0;
      }
      if (initObj.hasOwnProperty('phi_capture')) {
        this.phi_capture = initObj.phi_capture
      }
      else {
        this.phi_capture = 0.0;
      }
      if (initObj.hasOwnProperty('psi_capture')) {
        this.psi_capture = initObj.psi_capture
      }
      else {
        this.psi_capture = 0.0;
      }
      if (initObj.hasOwnProperty('altitude_capture')) {
        this.altitude_capture = initObj.altitude_capture
      }
      else {
        this.altitude_capture = 0;
      }
      if (initObj.hasOwnProperty('time_capture')) {
        this.time_capture = initObj.time_capture
      }
      else {
        this.time_capture = 0;
      }
      if (initObj.hasOwnProperty('body_v')) {
        this.body_v = initObj.body_v
      }
      else {
        this.body_v = new vector31();
      }
      if (initObj.hasOwnProperty('delta_phi')) {
        this.delta_phi = initObj.delta_phi
      }
      else {
        this.delta_phi = 0.0;
      }
      if (initObj.hasOwnProperty('delta_theta')) {
        this.delta_theta = initObj.delta_theta
      }
      else {
        this.delta_theta = 0.0;
      }
      if (initObj.hasOwnProperty('delta_psi')) {
        this.delta_psi = initObj.delta_psi
      }
      else {
        this.delta_psi = 0.0;
      }
      if (initObj.hasOwnProperty('gold_defined')) {
        this.gold_defined = initObj.gold_defined
      }
      else {
        this.gold_defined = 0;
      }
      if (initObj.hasOwnProperty('gold_reset')) {
        this.gold_reset = initObj.gold_reset
      }
      else {
        this.gold_reset = 0;
      }
      if (initObj.hasOwnProperty('gold_x')) {
        this.gold_x = initObj.gold_x
      }
      else {
        this.gold_x = 0.0;
      }
      if (initObj.hasOwnProperty('gold_y')) {
        this.gold_y = initObj.gold_y
      }
      else {
        this.gold_y = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type navdata_vision
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [drone_time]
    bufferOffset = _serializer.float64(obj.drone_time, buffer, bufferOffset);
    // Serialize message field [tag]
    bufferOffset = _serializer.uint16(obj.tag, buffer, bufferOffset);
    // Serialize message field [size]
    bufferOffset = _serializer.uint16(obj.size, buffer, bufferOffset);
    // Serialize message field [vision_state]
    bufferOffset = _serializer.uint32(obj.vision_state, buffer, bufferOffset);
    // Serialize message field [vision_misc]
    bufferOffset = _serializer.int32(obj.vision_misc, buffer, bufferOffset);
    // Serialize message field [vision_phi_trim]
    bufferOffset = _serializer.float32(obj.vision_phi_trim, buffer, bufferOffset);
    // Serialize message field [vision_phi_ref_prop]
    bufferOffset = _serializer.float32(obj.vision_phi_ref_prop, buffer, bufferOffset);
    // Serialize message field [vision_theta_trim]
    bufferOffset = _serializer.float32(obj.vision_theta_trim, buffer, bufferOffset);
    // Serialize message field [vision_theta_ref_prop]
    bufferOffset = _serializer.float32(obj.vision_theta_ref_prop, buffer, bufferOffset);
    // Serialize message field [new_raw_picture]
    bufferOffset = _serializer.int32(obj.new_raw_picture, buffer, bufferOffset);
    // Serialize message field [theta_capture]
    bufferOffset = _serializer.float32(obj.theta_capture, buffer, bufferOffset);
    // Serialize message field [phi_capture]
    bufferOffset = _serializer.float32(obj.phi_capture, buffer, bufferOffset);
    // Serialize message field [psi_capture]
    bufferOffset = _serializer.float32(obj.psi_capture, buffer, bufferOffset);
    // Serialize message field [altitude_capture]
    bufferOffset = _serializer.int32(obj.altitude_capture, buffer, bufferOffset);
    // Serialize message field [time_capture]
    bufferOffset = _serializer.uint32(obj.time_capture, buffer, bufferOffset);
    // Serialize message field [body_v]
    bufferOffset = vector31.serialize(obj.body_v, buffer, bufferOffset);
    // Serialize message field [delta_phi]
    bufferOffset = _serializer.float32(obj.delta_phi, buffer, bufferOffset);
    // Serialize message field [delta_theta]
    bufferOffset = _serializer.float32(obj.delta_theta, buffer, bufferOffset);
    // Serialize message field [delta_psi]
    bufferOffset = _serializer.float32(obj.delta_psi, buffer, bufferOffset);
    // Serialize message field [gold_defined]
    bufferOffset = _serializer.uint32(obj.gold_defined, buffer, bufferOffset);
    // Serialize message field [gold_reset]
    bufferOffset = _serializer.uint32(obj.gold_reset, buffer, bufferOffset);
    // Serialize message field [gold_x]
    bufferOffset = _serializer.float32(obj.gold_x, buffer, bufferOffset);
    // Serialize message field [gold_y]
    bufferOffset = _serializer.float32(obj.gold_y, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type navdata_vision
    let len;
    let data = new navdata_vision(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [drone_time]
    data.drone_time = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [tag]
    data.tag = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [size]
    data.size = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [vision_state]
    data.vision_state = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [vision_misc]
    data.vision_misc = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [vision_phi_trim]
    data.vision_phi_trim = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [vision_phi_ref_prop]
    data.vision_phi_ref_prop = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [vision_theta_trim]
    data.vision_theta_trim = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [vision_theta_ref_prop]
    data.vision_theta_ref_prop = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [new_raw_picture]
    data.new_raw_picture = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [theta_capture]
    data.theta_capture = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [phi_capture]
    data.phi_capture = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [psi_capture]
    data.psi_capture = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [altitude_capture]
    data.altitude_capture = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [time_capture]
    data.time_capture = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [body_v]
    data.body_v = vector31.deserialize(buffer, bufferOffset);
    // Deserialize message field [delta_phi]
    data.delta_phi = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [delta_theta]
    data.delta_theta = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [delta_psi]
    data.delta_psi = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [gold_defined]
    data.gold_defined = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [gold_reset]
    data.gold_reset = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [gold_x]
    data.gold_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [gold_y]
    data.gold_y = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 100;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ardrone_autonomy/navdata_vision';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '55dd9a6c13917332d6d39025ed1318ab';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    float64 drone_time
    uint16 tag
    uint16 size
    uint32 vision_state
    int32 vision_misc
    float32 vision_phi_trim
    float32 vision_phi_ref_prop
    float32 vision_theta_trim
    float32 vision_theta_ref_prop
    int32 new_raw_picture
    float32 theta_capture
    float32 phi_capture
    float32 psi_capture
    int32 altitude_capture
    uint32 time_capture
    vector31 body_v
    float32 delta_phi
    float32 delta_theta
    float32 delta_psi
    uint32 gold_defined
    uint32 gold_reset
    float32 gold_x
    float32 gold_y
    
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
    const resolved = new navdata_vision(null);
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

    if (msg.vision_state !== undefined) {
      resolved.vision_state = msg.vision_state;
    }
    else {
      resolved.vision_state = 0
    }

    if (msg.vision_misc !== undefined) {
      resolved.vision_misc = msg.vision_misc;
    }
    else {
      resolved.vision_misc = 0
    }

    if (msg.vision_phi_trim !== undefined) {
      resolved.vision_phi_trim = msg.vision_phi_trim;
    }
    else {
      resolved.vision_phi_trim = 0.0
    }

    if (msg.vision_phi_ref_prop !== undefined) {
      resolved.vision_phi_ref_prop = msg.vision_phi_ref_prop;
    }
    else {
      resolved.vision_phi_ref_prop = 0.0
    }

    if (msg.vision_theta_trim !== undefined) {
      resolved.vision_theta_trim = msg.vision_theta_trim;
    }
    else {
      resolved.vision_theta_trim = 0.0
    }

    if (msg.vision_theta_ref_prop !== undefined) {
      resolved.vision_theta_ref_prop = msg.vision_theta_ref_prop;
    }
    else {
      resolved.vision_theta_ref_prop = 0.0
    }

    if (msg.new_raw_picture !== undefined) {
      resolved.new_raw_picture = msg.new_raw_picture;
    }
    else {
      resolved.new_raw_picture = 0
    }

    if (msg.theta_capture !== undefined) {
      resolved.theta_capture = msg.theta_capture;
    }
    else {
      resolved.theta_capture = 0.0
    }

    if (msg.phi_capture !== undefined) {
      resolved.phi_capture = msg.phi_capture;
    }
    else {
      resolved.phi_capture = 0.0
    }

    if (msg.psi_capture !== undefined) {
      resolved.psi_capture = msg.psi_capture;
    }
    else {
      resolved.psi_capture = 0.0
    }

    if (msg.altitude_capture !== undefined) {
      resolved.altitude_capture = msg.altitude_capture;
    }
    else {
      resolved.altitude_capture = 0
    }

    if (msg.time_capture !== undefined) {
      resolved.time_capture = msg.time_capture;
    }
    else {
      resolved.time_capture = 0
    }

    if (msg.body_v !== undefined) {
      resolved.body_v = vector31.Resolve(msg.body_v)
    }
    else {
      resolved.body_v = new vector31()
    }

    if (msg.delta_phi !== undefined) {
      resolved.delta_phi = msg.delta_phi;
    }
    else {
      resolved.delta_phi = 0.0
    }

    if (msg.delta_theta !== undefined) {
      resolved.delta_theta = msg.delta_theta;
    }
    else {
      resolved.delta_theta = 0.0
    }

    if (msg.delta_psi !== undefined) {
      resolved.delta_psi = msg.delta_psi;
    }
    else {
      resolved.delta_psi = 0.0
    }

    if (msg.gold_defined !== undefined) {
      resolved.gold_defined = msg.gold_defined;
    }
    else {
      resolved.gold_defined = 0
    }

    if (msg.gold_reset !== undefined) {
      resolved.gold_reset = msg.gold_reset;
    }
    else {
      resolved.gold_reset = 0
    }

    if (msg.gold_x !== undefined) {
      resolved.gold_x = msg.gold_x;
    }
    else {
      resolved.gold_x = 0.0
    }

    if (msg.gold_y !== undefined) {
      resolved.gold_y = msg.gold_y;
    }
    else {
      resolved.gold_y = 0.0
    }

    return resolved;
    }
};

module.exports = navdata_vision;
