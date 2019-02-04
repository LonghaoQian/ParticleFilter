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

class navdata_gyros_offsets {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.drone_time = null;
      this.tag = null;
      this.size = null;
      this.offset_g = null;
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
      if (initObj.hasOwnProperty('offset_g')) {
        this.offset_g = initObj.offset_g
      }
      else {
        this.offset_g = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type navdata_gyros_offsets
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [drone_time]
    bufferOffset = _serializer.float64(obj.drone_time, buffer, bufferOffset);
    // Serialize message field [tag]
    bufferOffset = _serializer.uint16(obj.tag, buffer, bufferOffset);
    // Serialize message field [size]
    bufferOffset = _serializer.uint16(obj.size, buffer, bufferOffset);
    // Serialize message field [offset_g]
    bufferOffset = _arraySerializer.float32(obj.offset_g, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type navdata_gyros_offsets
    let len;
    let data = new navdata_gyros_offsets(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [drone_time]
    data.drone_time = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [tag]
    data.tag = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [size]
    data.size = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [offset_g]
    data.offset_g = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 4 * object.offset_g.length;
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ardrone_autonomy/navdata_gyros_offsets';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c1f7c76e172c058c98de03cda79060d4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    float64 drone_time
    uint16 tag
    uint16 size
    float32[] offset_g
    
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
    const resolved = new navdata_gyros_offsets(null);
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

    if (msg.offset_g !== undefined) {
      resolved.offset_g = msg.offset_g;
    }
    else {
      resolved.offset_g = []
    }

    return resolved;
    }
};

module.exports = navdata_gyros_offsets;
