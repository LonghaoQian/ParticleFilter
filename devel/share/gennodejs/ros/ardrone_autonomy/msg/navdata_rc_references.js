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

class navdata_rc_references {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.drone_time = null;
      this.tag = null;
      this.size = null;
      this.rc_ref_pitch = null;
      this.rc_ref_roll = null;
      this.rc_ref_yaw = null;
      this.rc_ref_gaz = null;
      this.rc_ref_ag = null;
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
      if (initObj.hasOwnProperty('rc_ref_pitch')) {
        this.rc_ref_pitch = initObj.rc_ref_pitch
      }
      else {
        this.rc_ref_pitch = 0;
      }
      if (initObj.hasOwnProperty('rc_ref_roll')) {
        this.rc_ref_roll = initObj.rc_ref_roll
      }
      else {
        this.rc_ref_roll = 0;
      }
      if (initObj.hasOwnProperty('rc_ref_yaw')) {
        this.rc_ref_yaw = initObj.rc_ref_yaw
      }
      else {
        this.rc_ref_yaw = 0;
      }
      if (initObj.hasOwnProperty('rc_ref_gaz')) {
        this.rc_ref_gaz = initObj.rc_ref_gaz
      }
      else {
        this.rc_ref_gaz = 0;
      }
      if (initObj.hasOwnProperty('rc_ref_ag')) {
        this.rc_ref_ag = initObj.rc_ref_ag
      }
      else {
        this.rc_ref_ag = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type navdata_rc_references
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [drone_time]
    bufferOffset = _serializer.float64(obj.drone_time, buffer, bufferOffset);
    // Serialize message field [tag]
    bufferOffset = _serializer.uint16(obj.tag, buffer, bufferOffset);
    // Serialize message field [size]
    bufferOffset = _serializer.uint16(obj.size, buffer, bufferOffset);
    // Serialize message field [rc_ref_pitch]
    bufferOffset = _serializer.int32(obj.rc_ref_pitch, buffer, bufferOffset);
    // Serialize message field [rc_ref_roll]
    bufferOffset = _serializer.int32(obj.rc_ref_roll, buffer, bufferOffset);
    // Serialize message field [rc_ref_yaw]
    bufferOffset = _serializer.int32(obj.rc_ref_yaw, buffer, bufferOffset);
    // Serialize message field [rc_ref_gaz]
    bufferOffset = _serializer.int32(obj.rc_ref_gaz, buffer, bufferOffset);
    // Serialize message field [rc_ref_ag]
    bufferOffset = _serializer.int32(obj.rc_ref_ag, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type navdata_rc_references
    let len;
    let data = new navdata_rc_references(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [drone_time]
    data.drone_time = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [tag]
    data.tag = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [size]
    data.size = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [rc_ref_pitch]
    data.rc_ref_pitch = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [rc_ref_roll]
    data.rc_ref_roll = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [rc_ref_yaw]
    data.rc_ref_yaw = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [rc_ref_gaz]
    data.rc_ref_gaz = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [rc_ref_ag]
    data.rc_ref_ag = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 32;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ardrone_autonomy/navdata_rc_references';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '57eebe7ce3b678ed3cfbdcf4646ffe93';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    float64 drone_time
    uint16 tag
    uint16 size
    int32 rc_ref_pitch
    int32 rc_ref_roll
    int32 rc_ref_yaw
    int32 rc_ref_gaz
    int32 rc_ref_ag
    
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
    const resolved = new navdata_rc_references(null);
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

    if (msg.rc_ref_pitch !== undefined) {
      resolved.rc_ref_pitch = msg.rc_ref_pitch;
    }
    else {
      resolved.rc_ref_pitch = 0
    }

    if (msg.rc_ref_roll !== undefined) {
      resolved.rc_ref_roll = msg.rc_ref_roll;
    }
    else {
      resolved.rc_ref_roll = 0
    }

    if (msg.rc_ref_yaw !== undefined) {
      resolved.rc_ref_yaw = msg.rc_ref_yaw;
    }
    else {
      resolved.rc_ref_yaw = 0
    }

    if (msg.rc_ref_gaz !== undefined) {
      resolved.rc_ref_gaz = msg.rc_ref_gaz;
    }
    else {
      resolved.rc_ref_gaz = 0
    }

    if (msg.rc_ref_ag !== undefined) {
      resolved.rc_ref_ag = msg.rc_ref_ag;
    }
    else {
      resolved.rc_ref_ag = 0
    }

    return resolved;
    }
};

module.exports = navdata_rc_references;
