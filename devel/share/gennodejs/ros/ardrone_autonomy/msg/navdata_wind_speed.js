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

class navdata_wind_speed {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.drone_time = null;
      this.tag = null;
      this.size = null;
      this.wind_speed = null;
      this.wind_angle = null;
      this.wind_compensation_theta = null;
      this.wind_compensation_phi = null;
      this.state_x1 = null;
      this.state_x2 = null;
      this.state_x3 = null;
      this.state_x4 = null;
      this.state_x5 = null;
      this.state_x6 = null;
      this.magneto_debug1 = null;
      this.magneto_debug2 = null;
      this.magneto_debug3 = null;
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
      if (initObj.hasOwnProperty('wind_speed')) {
        this.wind_speed = initObj.wind_speed
      }
      else {
        this.wind_speed = 0.0;
      }
      if (initObj.hasOwnProperty('wind_angle')) {
        this.wind_angle = initObj.wind_angle
      }
      else {
        this.wind_angle = 0.0;
      }
      if (initObj.hasOwnProperty('wind_compensation_theta')) {
        this.wind_compensation_theta = initObj.wind_compensation_theta
      }
      else {
        this.wind_compensation_theta = 0.0;
      }
      if (initObj.hasOwnProperty('wind_compensation_phi')) {
        this.wind_compensation_phi = initObj.wind_compensation_phi
      }
      else {
        this.wind_compensation_phi = 0.0;
      }
      if (initObj.hasOwnProperty('state_x1')) {
        this.state_x1 = initObj.state_x1
      }
      else {
        this.state_x1 = 0.0;
      }
      if (initObj.hasOwnProperty('state_x2')) {
        this.state_x2 = initObj.state_x2
      }
      else {
        this.state_x2 = 0.0;
      }
      if (initObj.hasOwnProperty('state_x3')) {
        this.state_x3 = initObj.state_x3
      }
      else {
        this.state_x3 = 0.0;
      }
      if (initObj.hasOwnProperty('state_x4')) {
        this.state_x4 = initObj.state_x4
      }
      else {
        this.state_x4 = 0.0;
      }
      if (initObj.hasOwnProperty('state_x5')) {
        this.state_x5 = initObj.state_x5
      }
      else {
        this.state_x5 = 0.0;
      }
      if (initObj.hasOwnProperty('state_x6')) {
        this.state_x6 = initObj.state_x6
      }
      else {
        this.state_x6 = 0.0;
      }
      if (initObj.hasOwnProperty('magneto_debug1')) {
        this.magneto_debug1 = initObj.magneto_debug1
      }
      else {
        this.magneto_debug1 = 0.0;
      }
      if (initObj.hasOwnProperty('magneto_debug2')) {
        this.magneto_debug2 = initObj.magneto_debug2
      }
      else {
        this.magneto_debug2 = 0.0;
      }
      if (initObj.hasOwnProperty('magneto_debug3')) {
        this.magneto_debug3 = initObj.magneto_debug3
      }
      else {
        this.magneto_debug3 = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type navdata_wind_speed
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [drone_time]
    bufferOffset = _serializer.float64(obj.drone_time, buffer, bufferOffset);
    // Serialize message field [tag]
    bufferOffset = _serializer.uint16(obj.tag, buffer, bufferOffset);
    // Serialize message field [size]
    bufferOffset = _serializer.uint16(obj.size, buffer, bufferOffset);
    // Serialize message field [wind_speed]
    bufferOffset = _serializer.float32(obj.wind_speed, buffer, bufferOffset);
    // Serialize message field [wind_angle]
    bufferOffset = _serializer.float32(obj.wind_angle, buffer, bufferOffset);
    // Serialize message field [wind_compensation_theta]
    bufferOffset = _serializer.float32(obj.wind_compensation_theta, buffer, bufferOffset);
    // Serialize message field [wind_compensation_phi]
    bufferOffset = _serializer.float32(obj.wind_compensation_phi, buffer, bufferOffset);
    // Serialize message field [state_x1]
    bufferOffset = _serializer.float32(obj.state_x1, buffer, bufferOffset);
    // Serialize message field [state_x2]
    bufferOffset = _serializer.float32(obj.state_x2, buffer, bufferOffset);
    // Serialize message field [state_x3]
    bufferOffset = _serializer.float32(obj.state_x3, buffer, bufferOffset);
    // Serialize message field [state_x4]
    bufferOffset = _serializer.float32(obj.state_x4, buffer, bufferOffset);
    // Serialize message field [state_x5]
    bufferOffset = _serializer.float32(obj.state_x5, buffer, bufferOffset);
    // Serialize message field [state_x6]
    bufferOffset = _serializer.float32(obj.state_x6, buffer, bufferOffset);
    // Serialize message field [magneto_debug1]
    bufferOffset = _serializer.float32(obj.magneto_debug1, buffer, bufferOffset);
    // Serialize message field [magneto_debug2]
    bufferOffset = _serializer.float32(obj.magneto_debug2, buffer, bufferOffset);
    // Serialize message field [magneto_debug3]
    bufferOffset = _serializer.float32(obj.magneto_debug3, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type navdata_wind_speed
    let len;
    let data = new navdata_wind_speed(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [drone_time]
    data.drone_time = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [tag]
    data.tag = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [size]
    data.size = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [wind_speed]
    data.wind_speed = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [wind_angle]
    data.wind_angle = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [wind_compensation_theta]
    data.wind_compensation_theta = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [wind_compensation_phi]
    data.wind_compensation_phi = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [state_x1]
    data.state_x1 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [state_x2]
    data.state_x2 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [state_x3]
    data.state_x3 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [state_x4]
    data.state_x4 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [state_x5]
    data.state_x5 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [state_x6]
    data.state_x6 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [magneto_debug1]
    data.magneto_debug1 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [magneto_debug2]
    data.magneto_debug2 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [magneto_debug3]
    data.magneto_debug3 = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 64;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ardrone_autonomy/navdata_wind_speed';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2cc5c1e9675c330dd38261e958a136d3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    float64 drone_time
    uint16 tag
    uint16 size
    float32 wind_speed
    float32 wind_angle
    float32 wind_compensation_theta
    float32 wind_compensation_phi
    float32 state_x1
    float32 state_x2
    float32 state_x3
    float32 state_x4
    float32 state_x5
    float32 state_x6
    float32 magneto_debug1
    float32 magneto_debug2
    float32 magneto_debug3
    
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
    const resolved = new navdata_wind_speed(null);
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

    if (msg.wind_speed !== undefined) {
      resolved.wind_speed = msg.wind_speed;
    }
    else {
      resolved.wind_speed = 0.0
    }

    if (msg.wind_angle !== undefined) {
      resolved.wind_angle = msg.wind_angle;
    }
    else {
      resolved.wind_angle = 0.0
    }

    if (msg.wind_compensation_theta !== undefined) {
      resolved.wind_compensation_theta = msg.wind_compensation_theta;
    }
    else {
      resolved.wind_compensation_theta = 0.0
    }

    if (msg.wind_compensation_phi !== undefined) {
      resolved.wind_compensation_phi = msg.wind_compensation_phi;
    }
    else {
      resolved.wind_compensation_phi = 0.0
    }

    if (msg.state_x1 !== undefined) {
      resolved.state_x1 = msg.state_x1;
    }
    else {
      resolved.state_x1 = 0.0
    }

    if (msg.state_x2 !== undefined) {
      resolved.state_x2 = msg.state_x2;
    }
    else {
      resolved.state_x2 = 0.0
    }

    if (msg.state_x3 !== undefined) {
      resolved.state_x3 = msg.state_x3;
    }
    else {
      resolved.state_x3 = 0.0
    }

    if (msg.state_x4 !== undefined) {
      resolved.state_x4 = msg.state_x4;
    }
    else {
      resolved.state_x4 = 0.0
    }

    if (msg.state_x5 !== undefined) {
      resolved.state_x5 = msg.state_x5;
    }
    else {
      resolved.state_x5 = 0.0
    }

    if (msg.state_x6 !== undefined) {
      resolved.state_x6 = msg.state_x6;
    }
    else {
      resolved.state_x6 = 0.0
    }

    if (msg.magneto_debug1 !== undefined) {
      resolved.magneto_debug1 = msg.magneto_debug1;
    }
    else {
      resolved.magneto_debug1 = 0.0
    }

    if (msg.magneto_debug2 !== undefined) {
      resolved.magneto_debug2 = msg.magneto_debug2;
    }
    else {
      resolved.magneto_debug2 = 0.0
    }

    if (msg.magneto_debug3 !== undefined) {
      resolved.magneto_debug3 = msg.magneto_debug3;
    }
    else {
      resolved.magneto_debug3 = 0.0
    }

    return resolved;
    }
};

module.exports = navdata_wind_speed;
