// Auto-generated. Do not edit!

// (in-package ardrone_autonomy.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let matrix33 = require('./matrix33.js');
let vector31 = require('./vector31.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class navdata_vision_detect {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.drone_time = null;
      this.tag = null;
      this.size = null;
      this.nb_detected = null;
      this.type = null;
      this.xc = null;
      this.yc = null;
      this.width = null;
      this.height = null;
      this.dist = null;
      this.orientation_angle = null;
      this.rotation = null;
      this.translation = null;
      this.camera_source = null;
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
      if (initObj.hasOwnProperty('nb_detected')) {
        this.nb_detected = initObj.nb_detected
      }
      else {
        this.nb_detected = 0;
      }
      if (initObj.hasOwnProperty('type')) {
        this.type = initObj.type
      }
      else {
        this.type = [];
      }
      if (initObj.hasOwnProperty('xc')) {
        this.xc = initObj.xc
      }
      else {
        this.xc = [];
      }
      if (initObj.hasOwnProperty('yc')) {
        this.yc = initObj.yc
      }
      else {
        this.yc = [];
      }
      if (initObj.hasOwnProperty('width')) {
        this.width = initObj.width
      }
      else {
        this.width = [];
      }
      if (initObj.hasOwnProperty('height')) {
        this.height = initObj.height
      }
      else {
        this.height = [];
      }
      if (initObj.hasOwnProperty('dist')) {
        this.dist = initObj.dist
      }
      else {
        this.dist = [];
      }
      if (initObj.hasOwnProperty('orientation_angle')) {
        this.orientation_angle = initObj.orientation_angle
      }
      else {
        this.orientation_angle = [];
      }
      if (initObj.hasOwnProperty('rotation')) {
        this.rotation = initObj.rotation
      }
      else {
        this.rotation = [];
      }
      if (initObj.hasOwnProperty('translation')) {
        this.translation = initObj.translation
      }
      else {
        this.translation = [];
      }
      if (initObj.hasOwnProperty('camera_source')) {
        this.camera_source = initObj.camera_source
      }
      else {
        this.camera_source = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type navdata_vision_detect
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [drone_time]
    bufferOffset = _serializer.float64(obj.drone_time, buffer, bufferOffset);
    // Serialize message field [tag]
    bufferOffset = _serializer.uint16(obj.tag, buffer, bufferOffset);
    // Serialize message field [size]
    bufferOffset = _serializer.uint16(obj.size, buffer, bufferOffset);
    // Serialize message field [nb_detected]
    bufferOffset = _serializer.uint32(obj.nb_detected, buffer, bufferOffset);
    // Serialize message field [type]
    bufferOffset = _arraySerializer.uint32(obj.type, buffer, bufferOffset, null);
    // Serialize message field [xc]
    bufferOffset = _arraySerializer.uint32(obj.xc, buffer, bufferOffset, null);
    // Serialize message field [yc]
    bufferOffset = _arraySerializer.uint32(obj.yc, buffer, bufferOffset, null);
    // Serialize message field [width]
    bufferOffset = _arraySerializer.uint32(obj.width, buffer, bufferOffset, null);
    // Serialize message field [height]
    bufferOffset = _arraySerializer.uint32(obj.height, buffer, bufferOffset, null);
    // Serialize message field [dist]
    bufferOffset = _arraySerializer.uint32(obj.dist, buffer, bufferOffset, null);
    // Serialize message field [orientation_angle]
    bufferOffset = _arraySerializer.float32(obj.orientation_angle, buffer, bufferOffset, null);
    // Serialize message field [rotation]
    // Serialize the length for message field [rotation]
    bufferOffset = _serializer.uint32(obj.rotation.length, buffer, bufferOffset);
    obj.rotation.forEach((val) => {
      bufferOffset = matrix33.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [translation]
    // Serialize the length for message field [translation]
    bufferOffset = _serializer.uint32(obj.translation.length, buffer, bufferOffset);
    obj.translation.forEach((val) => {
      bufferOffset = vector31.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [camera_source]
    bufferOffset = _arraySerializer.uint32(obj.camera_source, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type navdata_vision_detect
    let len;
    let data = new navdata_vision_detect(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [drone_time]
    data.drone_time = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [tag]
    data.tag = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [size]
    data.size = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [nb_detected]
    data.nb_detected = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [type]
    data.type = _arrayDeserializer.uint32(buffer, bufferOffset, null)
    // Deserialize message field [xc]
    data.xc = _arrayDeserializer.uint32(buffer, bufferOffset, null)
    // Deserialize message field [yc]
    data.yc = _arrayDeserializer.uint32(buffer, bufferOffset, null)
    // Deserialize message field [width]
    data.width = _arrayDeserializer.uint32(buffer, bufferOffset, null)
    // Deserialize message field [height]
    data.height = _arrayDeserializer.uint32(buffer, bufferOffset, null)
    // Deserialize message field [dist]
    data.dist = _arrayDeserializer.uint32(buffer, bufferOffset, null)
    // Deserialize message field [orientation_angle]
    data.orientation_angle = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [rotation]
    // Deserialize array length for message field [rotation]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.rotation = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.rotation[i] = matrix33.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [translation]
    // Deserialize array length for message field [translation]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.translation = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.translation[i] = vector31.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [camera_source]
    data.camera_source = _arrayDeserializer.uint32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 4 * object.type.length;
    length += 4 * object.xc.length;
    length += 4 * object.yc.length;
    length += 4 * object.width.length;
    length += 4 * object.height.length;
    length += 4 * object.dist.length;
    length += 4 * object.orientation_angle.length;
    length += 36 * object.rotation.length;
    length += 12 * object.translation.length;
    length += 4 * object.camera_source.length;
    return length + 56;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ardrone_autonomy/navdata_vision_detect';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ab5f6b8c66aead58358d45a7c2dce7f6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    float64 drone_time
    uint16 tag
    uint16 size
    uint32 nb_detected
    uint32[] type
    uint32[] xc
    uint32[] yc
    uint32[] width
    uint32[] height
    uint32[] dist
    float32[] orientation_angle
    matrix33[] rotation
    vector31[] translation
    uint32[] camera_source
    
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
    MSG: ardrone_autonomy/matrix33
    float32 m11
    float32 m12
    float32 m13
    float32 m21
    float32 m22
    float32 m23
    float32 m31
    float32 m32
    float32 m33
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
    const resolved = new navdata_vision_detect(null);
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

    if (msg.nb_detected !== undefined) {
      resolved.nb_detected = msg.nb_detected;
    }
    else {
      resolved.nb_detected = 0
    }

    if (msg.type !== undefined) {
      resolved.type = msg.type;
    }
    else {
      resolved.type = []
    }

    if (msg.xc !== undefined) {
      resolved.xc = msg.xc;
    }
    else {
      resolved.xc = []
    }

    if (msg.yc !== undefined) {
      resolved.yc = msg.yc;
    }
    else {
      resolved.yc = []
    }

    if (msg.width !== undefined) {
      resolved.width = msg.width;
    }
    else {
      resolved.width = []
    }

    if (msg.height !== undefined) {
      resolved.height = msg.height;
    }
    else {
      resolved.height = []
    }

    if (msg.dist !== undefined) {
      resolved.dist = msg.dist;
    }
    else {
      resolved.dist = []
    }

    if (msg.orientation_angle !== undefined) {
      resolved.orientation_angle = msg.orientation_angle;
    }
    else {
      resolved.orientation_angle = []
    }

    if (msg.rotation !== undefined) {
      resolved.rotation = new Array(msg.rotation.length);
      for (let i = 0; i < resolved.rotation.length; ++i) {
        resolved.rotation[i] = matrix33.Resolve(msg.rotation[i]);
      }
    }
    else {
      resolved.rotation = []
    }

    if (msg.translation !== undefined) {
      resolved.translation = new Array(msg.translation.length);
      for (let i = 0; i < resolved.translation.length; ++i) {
        resolved.translation[i] = vector31.Resolve(msg.translation[i]);
      }
    }
    else {
      resolved.translation = []
    }

    if (msg.camera_source !== undefined) {
      resolved.camera_source = msg.camera_source;
    }
    else {
      resolved.camera_source = []
    }

    return resolved;
    }
};

module.exports = navdata_vision_detect;
