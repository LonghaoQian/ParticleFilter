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

class navdata_vision_perf {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.drone_time = null;
      this.tag = null;
      this.size = null;
      this.time_corners = null;
      this.time_compute = null;
      this.time_tracking = null;
      this.time_trans = null;
      this.time_update = null;
      this.time_custom = null;
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
      if (initObj.hasOwnProperty('time_corners')) {
        this.time_corners = initObj.time_corners
      }
      else {
        this.time_corners = 0.0;
      }
      if (initObj.hasOwnProperty('time_compute')) {
        this.time_compute = initObj.time_compute
      }
      else {
        this.time_compute = 0.0;
      }
      if (initObj.hasOwnProperty('time_tracking')) {
        this.time_tracking = initObj.time_tracking
      }
      else {
        this.time_tracking = 0.0;
      }
      if (initObj.hasOwnProperty('time_trans')) {
        this.time_trans = initObj.time_trans
      }
      else {
        this.time_trans = 0.0;
      }
      if (initObj.hasOwnProperty('time_update')) {
        this.time_update = initObj.time_update
      }
      else {
        this.time_update = 0.0;
      }
      if (initObj.hasOwnProperty('time_custom')) {
        this.time_custom = initObj.time_custom
      }
      else {
        this.time_custom = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type navdata_vision_perf
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [drone_time]
    bufferOffset = _serializer.float64(obj.drone_time, buffer, bufferOffset);
    // Serialize message field [tag]
    bufferOffset = _serializer.uint16(obj.tag, buffer, bufferOffset);
    // Serialize message field [size]
    bufferOffset = _serializer.uint16(obj.size, buffer, bufferOffset);
    // Serialize message field [time_corners]
    bufferOffset = _serializer.float32(obj.time_corners, buffer, bufferOffset);
    // Serialize message field [time_compute]
    bufferOffset = _serializer.float32(obj.time_compute, buffer, bufferOffset);
    // Serialize message field [time_tracking]
    bufferOffset = _serializer.float32(obj.time_tracking, buffer, bufferOffset);
    // Serialize message field [time_trans]
    bufferOffset = _serializer.float32(obj.time_trans, buffer, bufferOffset);
    // Serialize message field [time_update]
    bufferOffset = _serializer.float32(obj.time_update, buffer, bufferOffset);
    // Serialize message field [time_custom]
    bufferOffset = _arraySerializer.float32(obj.time_custom, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type navdata_vision_perf
    let len;
    let data = new navdata_vision_perf(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [drone_time]
    data.drone_time = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [tag]
    data.tag = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [size]
    data.size = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [time_corners]
    data.time_corners = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [time_compute]
    data.time_compute = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [time_tracking]
    data.time_tracking = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [time_trans]
    data.time_trans = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [time_update]
    data.time_update = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [time_custom]
    data.time_custom = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 4 * object.time_custom.length;
    return length + 36;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ardrone_autonomy/navdata_vision_perf';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5ed8267a2e2980a430a3439af8e5c9f7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    float64 drone_time
    uint16 tag
    uint16 size
    float32 time_corners
    float32 time_compute
    float32 time_tracking
    float32 time_trans
    float32 time_update
    float32[] time_custom
    
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
    const resolved = new navdata_vision_perf(null);
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

    if (msg.time_corners !== undefined) {
      resolved.time_corners = msg.time_corners;
    }
    else {
      resolved.time_corners = 0.0
    }

    if (msg.time_compute !== undefined) {
      resolved.time_compute = msg.time_compute;
    }
    else {
      resolved.time_compute = 0.0
    }

    if (msg.time_tracking !== undefined) {
      resolved.time_tracking = msg.time_tracking;
    }
    else {
      resolved.time_tracking = 0.0
    }

    if (msg.time_trans !== undefined) {
      resolved.time_trans = msg.time_trans;
    }
    else {
      resolved.time_trans = 0.0
    }

    if (msg.time_update !== undefined) {
      resolved.time_update = msg.time_update;
    }
    else {
      resolved.time_update = 0.0
    }

    if (msg.time_custom !== undefined) {
      resolved.time_custom = msg.time_custom;
    }
    else {
      resolved.time_custom = []
    }

    return resolved;
    }
};

module.exports = navdata_vision_perf;
