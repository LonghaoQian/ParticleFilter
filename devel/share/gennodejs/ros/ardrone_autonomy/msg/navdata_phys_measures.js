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

class navdata_phys_measures {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.drone_time = null;
      this.tag = null;
      this.size = null;
      this.accs_temp = null;
      this.gyro_temp = null;
      this.phys_accs = null;
      this.phys_gyros = null;
      this.alim3V3 = null;
      this.vrefEpson = null;
      this.vrefIDG = null;
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
      if (initObj.hasOwnProperty('accs_temp')) {
        this.accs_temp = initObj.accs_temp
      }
      else {
        this.accs_temp = 0.0;
      }
      if (initObj.hasOwnProperty('gyro_temp')) {
        this.gyro_temp = initObj.gyro_temp
      }
      else {
        this.gyro_temp = 0;
      }
      if (initObj.hasOwnProperty('phys_accs')) {
        this.phys_accs = initObj.phys_accs
      }
      else {
        this.phys_accs = [];
      }
      if (initObj.hasOwnProperty('phys_gyros')) {
        this.phys_gyros = initObj.phys_gyros
      }
      else {
        this.phys_gyros = [];
      }
      if (initObj.hasOwnProperty('alim3V3')) {
        this.alim3V3 = initObj.alim3V3
      }
      else {
        this.alim3V3 = 0;
      }
      if (initObj.hasOwnProperty('vrefEpson')) {
        this.vrefEpson = initObj.vrefEpson
      }
      else {
        this.vrefEpson = 0;
      }
      if (initObj.hasOwnProperty('vrefIDG')) {
        this.vrefIDG = initObj.vrefIDG
      }
      else {
        this.vrefIDG = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type navdata_phys_measures
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [drone_time]
    bufferOffset = _serializer.float64(obj.drone_time, buffer, bufferOffset);
    // Serialize message field [tag]
    bufferOffset = _serializer.uint16(obj.tag, buffer, bufferOffset);
    // Serialize message field [size]
    bufferOffset = _serializer.uint16(obj.size, buffer, bufferOffset);
    // Serialize message field [accs_temp]
    bufferOffset = _serializer.float32(obj.accs_temp, buffer, bufferOffset);
    // Serialize message field [gyro_temp]
    bufferOffset = _serializer.uint16(obj.gyro_temp, buffer, bufferOffset);
    // Serialize message field [phys_accs]
    bufferOffset = _arraySerializer.float32(obj.phys_accs, buffer, bufferOffset, null);
    // Serialize message field [phys_gyros]
    bufferOffset = _arraySerializer.float32(obj.phys_gyros, buffer, bufferOffset, null);
    // Serialize message field [alim3V3]
    bufferOffset = _serializer.uint32(obj.alim3V3, buffer, bufferOffset);
    // Serialize message field [vrefEpson]
    bufferOffset = _serializer.uint32(obj.vrefEpson, buffer, bufferOffset);
    // Serialize message field [vrefIDG]
    bufferOffset = _serializer.uint32(obj.vrefIDG, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type navdata_phys_measures
    let len;
    let data = new navdata_phys_measures(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [drone_time]
    data.drone_time = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [tag]
    data.tag = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [size]
    data.size = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [accs_temp]
    data.accs_temp = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [gyro_temp]
    data.gyro_temp = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [phys_accs]
    data.phys_accs = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [phys_gyros]
    data.phys_gyros = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [alim3V3]
    data.alim3V3 = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [vrefEpson]
    data.vrefEpson = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [vrefIDG]
    data.vrefIDG = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 4 * object.phys_accs.length;
    length += 4 * object.phys_gyros.length;
    return length + 38;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ardrone_autonomy/navdata_phys_measures';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '649dd469618f5ff313f6bceab2c356b7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    float64 drone_time
    uint16 tag
    uint16 size
    float32 accs_temp
    uint16 gyro_temp
    float32[] phys_accs
    float32[] phys_gyros
    uint32 alim3V3
    uint32 vrefEpson
    uint32 vrefIDG
    
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
    const resolved = new navdata_phys_measures(null);
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

    if (msg.accs_temp !== undefined) {
      resolved.accs_temp = msg.accs_temp;
    }
    else {
      resolved.accs_temp = 0.0
    }

    if (msg.gyro_temp !== undefined) {
      resolved.gyro_temp = msg.gyro_temp;
    }
    else {
      resolved.gyro_temp = 0
    }

    if (msg.phys_accs !== undefined) {
      resolved.phys_accs = msg.phys_accs;
    }
    else {
      resolved.phys_accs = []
    }

    if (msg.phys_gyros !== undefined) {
      resolved.phys_gyros = msg.phys_gyros;
    }
    else {
      resolved.phys_gyros = []
    }

    if (msg.alim3V3 !== undefined) {
      resolved.alim3V3 = msg.alim3V3;
    }
    else {
      resolved.alim3V3 = 0
    }

    if (msg.vrefEpson !== undefined) {
      resolved.vrefEpson = msg.vrefEpson;
    }
    else {
      resolved.vrefEpson = 0
    }

    if (msg.vrefIDG !== undefined) {
      resolved.vrefIDG = msg.vrefIDG;
    }
    else {
      resolved.vrefIDG = 0
    }

    return resolved;
    }
};

module.exports = navdata_phys_measures;
