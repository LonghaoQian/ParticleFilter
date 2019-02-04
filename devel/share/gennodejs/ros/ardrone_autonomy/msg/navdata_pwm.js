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

class navdata_pwm {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.drone_time = null;
      this.tag = null;
      this.size = null;
      this.motor1 = null;
      this.motor2 = null;
      this.motor3 = null;
      this.motor4 = null;
      this.sat_motor1 = null;
      this.sat_motor2 = null;
      this.sat_motor3 = null;
      this.sat_motor4 = null;
      this.gaz_feed_forward = null;
      this.gaz_altitude = null;
      this.altitude_integral = null;
      this.vz_ref = null;
      this.u_pitch = null;
      this.u_roll = null;
      this.u_yaw = null;
      this.yaw_u_I = null;
      this.u_pitch_planif = null;
      this.u_roll_planif = null;
      this.u_yaw_planif = null;
      this.u_gaz_planif = null;
      this.current_motor1 = null;
      this.current_motor2 = null;
      this.current_motor3 = null;
      this.current_motor4 = null;
      this.altitude_der = null;
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
      if (initObj.hasOwnProperty('motor1')) {
        this.motor1 = initObj.motor1
      }
      else {
        this.motor1 = 0;
      }
      if (initObj.hasOwnProperty('motor2')) {
        this.motor2 = initObj.motor2
      }
      else {
        this.motor2 = 0;
      }
      if (initObj.hasOwnProperty('motor3')) {
        this.motor3 = initObj.motor3
      }
      else {
        this.motor3 = 0;
      }
      if (initObj.hasOwnProperty('motor4')) {
        this.motor4 = initObj.motor4
      }
      else {
        this.motor4 = 0;
      }
      if (initObj.hasOwnProperty('sat_motor1')) {
        this.sat_motor1 = initObj.sat_motor1
      }
      else {
        this.sat_motor1 = 0;
      }
      if (initObj.hasOwnProperty('sat_motor2')) {
        this.sat_motor2 = initObj.sat_motor2
      }
      else {
        this.sat_motor2 = 0;
      }
      if (initObj.hasOwnProperty('sat_motor3')) {
        this.sat_motor3 = initObj.sat_motor3
      }
      else {
        this.sat_motor3 = 0;
      }
      if (initObj.hasOwnProperty('sat_motor4')) {
        this.sat_motor4 = initObj.sat_motor4
      }
      else {
        this.sat_motor4 = 0;
      }
      if (initObj.hasOwnProperty('gaz_feed_forward')) {
        this.gaz_feed_forward = initObj.gaz_feed_forward
      }
      else {
        this.gaz_feed_forward = 0.0;
      }
      if (initObj.hasOwnProperty('gaz_altitude')) {
        this.gaz_altitude = initObj.gaz_altitude
      }
      else {
        this.gaz_altitude = 0.0;
      }
      if (initObj.hasOwnProperty('altitude_integral')) {
        this.altitude_integral = initObj.altitude_integral
      }
      else {
        this.altitude_integral = 0.0;
      }
      if (initObj.hasOwnProperty('vz_ref')) {
        this.vz_ref = initObj.vz_ref
      }
      else {
        this.vz_ref = 0.0;
      }
      if (initObj.hasOwnProperty('u_pitch')) {
        this.u_pitch = initObj.u_pitch
      }
      else {
        this.u_pitch = 0;
      }
      if (initObj.hasOwnProperty('u_roll')) {
        this.u_roll = initObj.u_roll
      }
      else {
        this.u_roll = 0;
      }
      if (initObj.hasOwnProperty('u_yaw')) {
        this.u_yaw = initObj.u_yaw
      }
      else {
        this.u_yaw = 0;
      }
      if (initObj.hasOwnProperty('yaw_u_I')) {
        this.yaw_u_I = initObj.yaw_u_I
      }
      else {
        this.yaw_u_I = 0.0;
      }
      if (initObj.hasOwnProperty('u_pitch_planif')) {
        this.u_pitch_planif = initObj.u_pitch_planif
      }
      else {
        this.u_pitch_planif = 0;
      }
      if (initObj.hasOwnProperty('u_roll_planif')) {
        this.u_roll_planif = initObj.u_roll_planif
      }
      else {
        this.u_roll_planif = 0;
      }
      if (initObj.hasOwnProperty('u_yaw_planif')) {
        this.u_yaw_planif = initObj.u_yaw_planif
      }
      else {
        this.u_yaw_planif = 0;
      }
      if (initObj.hasOwnProperty('u_gaz_planif')) {
        this.u_gaz_planif = initObj.u_gaz_planif
      }
      else {
        this.u_gaz_planif = 0.0;
      }
      if (initObj.hasOwnProperty('current_motor1')) {
        this.current_motor1 = initObj.current_motor1
      }
      else {
        this.current_motor1 = 0;
      }
      if (initObj.hasOwnProperty('current_motor2')) {
        this.current_motor2 = initObj.current_motor2
      }
      else {
        this.current_motor2 = 0;
      }
      if (initObj.hasOwnProperty('current_motor3')) {
        this.current_motor3 = initObj.current_motor3
      }
      else {
        this.current_motor3 = 0;
      }
      if (initObj.hasOwnProperty('current_motor4')) {
        this.current_motor4 = initObj.current_motor4
      }
      else {
        this.current_motor4 = 0;
      }
      if (initObj.hasOwnProperty('altitude_der')) {
        this.altitude_der = initObj.altitude_der
      }
      else {
        this.altitude_der = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type navdata_pwm
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [drone_time]
    bufferOffset = _serializer.float64(obj.drone_time, buffer, bufferOffset);
    // Serialize message field [tag]
    bufferOffset = _serializer.uint16(obj.tag, buffer, bufferOffset);
    // Serialize message field [size]
    bufferOffset = _serializer.uint16(obj.size, buffer, bufferOffset);
    // Serialize message field [motor1]
    bufferOffset = _serializer.uint8(obj.motor1, buffer, bufferOffset);
    // Serialize message field [motor2]
    bufferOffset = _serializer.uint8(obj.motor2, buffer, bufferOffset);
    // Serialize message field [motor3]
    bufferOffset = _serializer.uint8(obj.motor3, buffer, bufferOffset);
    // Serialize message field [motor4]
    bufferOffset = _serializer.uint8(obj.motor4, buffer, bufferOffset);
    // Serialize message field [sat_motor1]
    bufferOffset = _serializer.uint8(obj.sat_motor1, buffer, bufferOffset);
    // Serialize message field [sat_motor2]
    bufferOffset = _serializer.uint8(obj.sat_motor2, buffer, bufferOffset);
    // Serialize message field [sat_motor3]
    bufferOffset = _serializer.uint8(obj.sat_motor3, buffer, bufferOffset);
    // Serialize message field [sat_motor4]
    bufferOffset = _serializer.uint8(obj.sat_motor4, buffer, bufferOffset);
    // Serialize message field [gaz_feed_forward]
    bufferOffset = _serializer.float32(obj.gaz_feed_forward, buffer, bufferOffset);
    // Serialize message field [gaz_altitude]
    bufferOffset = _serializer.float32(obj.gaz_altitude, buffer, bufferOffset);
    // Serialize message field [altitude_integral]
    bufferOffset = _serializer.float32(obj.altitude_integral, buffer, bufferOffset);
    // Serialize message field [vz_ref]
    bufferOffset = _serializer.float32(obj.vz_ref, buffer, bufferOffset);
    // Serialize message field [u_pitch]
    bufferOffset = _serializer.int32(obj.u_pitch, buffer, bufferOffset);
    // Serialize message field [u_roll]
    bufferOffset = _serializer.int32(obj.u_roll, buffer, bufferOffset);
    // Serialize message field [u_yaw]
    bufferOffset = _serializer.int32(obj.u_yaw, buffer, bufferOffset);
    // Serialize message field [yaw_u_I]
    bufferOffset = _serializer.float32(obj.yaw_u_I, buffer, bufferOffset);
    // Serialize message field [u_pitch_planif]
    bufferOffset = _serializer.int32(obj.u_pitch_planif, buffer, bufferOffset);
    // Serialize message field [u_roll_planif]
    bufferOffset = _serializer.int32(obj.u_roll_planif, buffer, bufferOffset);
    // Serialize message field [u_yaw_planif]
    bufferOffset = _serializer.int32(obj.u_yaw_planif, buffer, bufferOffset);
    // Serialize message field [u_gaz_planif]
    bufferOffset = _serializer.float32(obj.u_gaz_planif, buffer, bufferOffset);
    // Serialize message field [current_motor1]
    bufferOffset = _serializer.uint16(obj.current_motor1, buffer, bufferOffset);
    // Serialize message field [current_motor2]
    bufferOffset = _serializer.uint16(obj.current_motor2, buffer, bufferOffset);
    // Serialize message field [current_motor3]
    bufferOffset = _serializer.uint16(obj.current_motor3, buffer, bufferOffset);
    // Serialize message field [current_motor4]
    bufferOffset = _serializer.uint16(obj.current_motor4, buffer, bufferOffset);
    // Serialize message field [altitude_der]
    bufferOffset = _serializer.float32(obj.altitude_der, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type navdata_pwm
    let len;
    let data = new navdata_pwm(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [drone_time]
    data.drone_time = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [tag]
    data.tag = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [size]
    data.size = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [motor1]
    data.motor1 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [motor2]
    data.motor2 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [motor3]
    data.motor3 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [motor4]
    data.motor4 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [sat_motor1]
    data.sat_motor1 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [sat_motor2]
    data.sat_motor2 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [sat_motor3]
    data.sat_motor3 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [sat_motor4]
    data.sat_motor4 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [gaz_feed_forward]
    data.gaz_feed_forward = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [gaz_altitude]
    data.gaz_altitude = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [altitude_integral]
    data.altitude_integral = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [vz_ref]
    data.vz_ref = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [u_pitch]
    data.u_pitch = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [u_roll]
    data.u_roll = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [u_yaw]
    data.u_yaw = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [yaw_u_I]
    data.yaw_u_I = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [u_pitch_planif]
    data.u_pitch_planif = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [u_roll_planif]
    data.u_roll_planif = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [u_yaw_planif]
    data.u_yaw_planif = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [u_gaz_planif]
    data.u_gaz_planif = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [current_motor1]
    data.current_motor1 = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [current_motor2]
    data.current_motor2 = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [current_motor3]
    data.current_motor3 = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [current_motor4]
    data.current_motor4 = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [altitude_der]
    data.altitude_der = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 80;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ardrone_autonomy/navdata_pwm';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6369e637a869960c68d7de5c2e10d6b1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    float64 drone_time
    uint16 tag
    uint16 size
    uint8 motor1
    uint8 motor2
    uint8 motor3
    uint8 motor4
    uint8 sat_motor1
    uint8 sat_motor2
    uint8 sat_motor3
    uint8 sat_motor4
    float32 gaz_feed_forward
    float32 gaz_altitude
    float32 altitude_integral
    float32 vz_ref
    int32 u_pitch
    int32 u_roll
    int32 u_yaw
    float32 yaw_u_I
    int32 u_pitch_planif
    int32 u_roll_planif
    int32 u_yaw_planif
    float32 u_gaz_planif
    uint16 current_motor1
    uint16 current_motor2
    uint16 current_motor3
    uint16 current_motor4
    float32 altitude_der
    
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
    const resolved = new navdata_pwm(null);
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

    if (msg.motor1 !== undefined) {
      resolved.motor1 = msg.motor1;
    }
    else {
      resolved.motor1 = 0
    }

    if (msg.motor2 !== undefined) {
      resolved.motor2 = msg.motor2;
    }
    else {
      resolved.motor2 = 0
    }

    if (msg.motor3 !== undefined) {
      resolved.motor3 = msg.motor3;
    }
    else {
      resolved.motor3 = 0
    }

    if (msg.motor4 !== undefined) {
      resolved.motor4 = msg.motor4;
    }
    else {
      resolved.motor4 = 0
    }

    if (msg.sat_motor1 !== undefined) {
      resolved.sat_motor1 = msg.sat_motor1;
    }
    else {
      resolved.sat_motor1 = 0
    }

    if (msg.sat_motor2 !== undefined) {
      resolved.sat_motor2 = msg.sat_motor2;
    }
    else {
      resolved.sat_motor2 = 0
    }

    if (msg.sat_motor3 !== undefined) {
      resolved.sat_motor3 = msg.sat_motor3;
    }
    else {
      resolved.sat_motor3 = 0
    }

    if (msg.sat_motor4 !== undefined) {
      resolved.sat_motor4 = msg.sat_motor4;
    }
    else {
      resolved.sat_motor4 = 0
    }

    if (msg.gaz_feed_forward !== undefined) {
      resolved.gaz_feed_forward = msg.gaz_feed_forward;
    }
    else {
      resolved.gaz_feed_forward = 0.0
    }

    if (msg.gaz_altitude !== undefined) {
      resolved.gaz_altitude = msg.gaz_altitude;
    }
    else {
      resolved.gaz_altitude = 0.0
    }

    if (msg.altitude_integral !== undefined) {
      resolved.altitude_integral = msg.altitude_integral;
    }
    else {
      resolved.altitude_integral = 0.0
    }

    if (msg.vz_ref !== undefined) {
      resolved.vz_ref = msg.vz_ref;
    }
    else {
      resolved.vz_ref = 0.0
    }

    if (msg.u_pitch !== undefined) {
      resolved.u_pitch = msg.u_pitch;
    }
    else {
      resolved.u_pitch = 0
    }

    if (msg.u_roll !== undefined) {
      resolved.u_roll = msg.u_roll;
    }
    else {
      resolved.u_roll = 0
    }

    if (msg.u_yaw !== undefined) {
      resolved.u_yaw = msg.u_yaw;
    }
    else {
      resolved.u_yaw = 0
    }

    if (msg.yaw_u_I !== undefined) {
      resolved.yaw_u_I = msg.yaw_u_I;
    }
    else {
      resolved.yaw_u_I = 0.0
    }

    if (msg.u_pitch_planif !== undefined) {
      resolved.u_pitch_planif = msg.u_pitch_planif;
    }
    else {
      resolved.u_pitch_planif = 0
    }

    if (msg.u_roll_planif !== undefined) {
      resolved.u_roll_planif = msg.u_roll_planif;
    }
    else {
      resolved.u_roll_planif = 0
    }

    if (msg.u_yaw_planif !== undefined) {
      resolved.u_yaw_planif = msg.u_yaw_planif;
    }
    else {
      resolved.u_yaw_planif = 0
    }

    if (msg.u_gaz_planif !== undefined) {
      resolved.u_gaz_planif = msg.u_gaz_planif;
    }
    else {
      resolved.u_gaz_planif = 0.0
    }

    if (msg.current_motor1 !== undefined) {
      resolved.current_motor1 = msg.current_motor1;
    }
    else {
      resolved.current_motor1 = 0
    }

    if (msg.current_motor2 !== undefined) {
      resolved.current_motor2 = msg.current_motor2;
    }
    else {
      resolved.current_motor2 = 0
    }

    if (msg.current_motor3 !== undefined) {
      resolved.current_motor3 = msg.current_motor3;
    }
    else {
      resolved.current_motor3 = 0
    }

    if (msg.current_motor4 !== undefined) {
      resolved.current_motor4 = msg.current_motor4;
    }
    else {
      resolved.current_motor4 = 0
    }

    if (msg.altitude_der !== undefined) {
      resolved.altitude_der = msg.altitude_der;
    }
    else {
      resolved.altitude_der = 0.0
    }

    return resolved;
    }
};

module.exports = navdata_pwm;
