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

class navdata_video_stream {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.drone_time = null;
      this.tag = null;
      this.size = null;
      this.quant = null;
      this.frame_size = null;
      this.frame_number = null;
      this.atcmd_ref_seq = null;
      this.atcmd_mean_ref_gap = null;
      this.atcmd_var_ref_gap = null;
      this.atcmd_ref_quality = null;
      this.desired_bitrate = null;
      this.data2 = null;
      this.data3 = null;
      this.data4 = null;
      this.data5 = null;
      this.fifo_queue_level = null;
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
      if (initObj.hasOwnProperty('quant')) {
        this.quant = initObj.quant
      }
      else {
        this.quant = 0;
      }
      if (initObj.hasOwnProperty('frame_size')) {
        this.frame_size = initObj.frame_size
      }
      else {
        this.frame_size = 0;
      }
      if (initObj.hasOwnProperty('frame_number')) {
        this.frame_number = initObj.frame_number
      }
      else {
        this.frame_number = 0;
      }
      if (initObj.hasOwnProperty('atcmd_ref_seq')) {
        this.atcmd_ref_seq = initObj.atcmd_ref_seq
      }
      else {
        this.atcmd_ref_seq = 0;
      }
      if (initObj.hasOwnProperty('atcmd_mean_ref_gap')) {
        this.atcmd_mean_ref_gap = initObj.atcmd_mean_ref_gap
      }
      else {
        this.atcmd_mean_ref_gap = 0;
      }
      if (initObj.hasOwnProperty('atcmd_var_ref_gap')) {
        this.atcmd_var_ref_gap = initObj.atcmd_var_ref_gap
      }
      else {
        this.atcmd_var_ref_gap = 0.0;
      }
      if (initObj.hasOwnProperty('atcmd_ref_quality')) {
        this.atcmd_ref_quality = initObj.atcmd_ref_quality
      }
      else {
        this.atcmd_ref_quality = 0;
      }
      if (initObj.hasOwnProperty('desired_bitrate')) {
        this.desired_bitrate = initObj.desired_bitrate
      }
      else {
        this.desired_bitrate = 0;
      }
      if (initObj.hasOwnProperty('data2')) {
        this.data2 = initObj.data2
      }
      else {
        this.data2 = 0;
      }
      if (initObj.hasOwnProperty('data3')) {
        this.data3 = initObj.data3
      }
      else {
        this.data3 = 0;
      }
      if (initObj.hasOwnProperty('data4')) {
        this.data4 = initObj.data4
      }
      else {
        this.data4 = 0;
      }
      if (initObj.hasOwnProperty('data5')) {
        this.data5 = initObj.data5
      }
      else {
        this.data5 = 0;
      }
      if (initObj.hasOwnProperty('fifo_queue_level')) {
        this.fifo_queue_level = initObj.fifo_queue_level
      }
      else {
        this.fifo_queue_level = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type navdata_video_stream
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [drone_time]
    bufferOffset = _serializer.float64(obj.drone_time, buffer, bufferOffset);
    // Serialize message field [tag]
    bufferOffset = _serializer.uint16(obj.tag, buffer, bufferOffset);
    // Serialize message field [size]
    bufferOffset = _serializer.uint16(obj.size, buffer, bufferOffset);
    // Serialize message field [quant]
    bufferOffset = _serializer.uint8(obj.quant, buffer, bufferOffset);
    // Serialize message field [frame_size]
    bufferOffset = _serializer.uint32(obj.frame_size, buffer, bufferOffset);
    // Serialize message field [frame_number]
    bufferOffset = _serializer.uint32(obj.frame_number, buffer, bufferOffset);
    // Serialize message field [atcmd_ref_seq]
    bufferOffset = _serializer.uint32(obj.atcmd_ref_seq, buffer, bufferOffset);
    // Serialize message field [atcmd_mean_ref_gap]
    bufferOffset = _serializer.uint32(obj.atcmd_mean_ref_gap, buffer, bufferOffset);
    // Serialize message field [atcmd_var_ref_gap]
    bufferOffset = _serializer.float32(obj.atcmd_var_ref_gap, buffer, bufferOffset);
    // Serialize message field [atcmd_ref_quality]
    bufferOffset = _serializer.uint32(obj.atcmd_ref_quality, buffer, bufferOffset);
    // Serialize message field [desired_bitrate]
    bufferOffset = _serializer.uint32(obj.desired_bitrate, buffer, bufferOffset);
    // Serialize message field [data2]
    bufferOffset = _serializer.int32(obj.data2, buffer, bufferOffset);
    // Serialize message field [data3]
    bufferOffset = _serializer.int32(obj.data3, buffer, bufferOffset);
    // Serialize message field [data4]
    bufferOffset = _serializer.int32(obj.data4, buffer, bufferOffset);
    // Serialize message field [data5]
    bufferOffset = _serializer.int32(obj.data5, buffer, bufferOffset);
    // Serialize message field [fifo_queue_level]
    bufferOffset = _serializer.uint32(obj.fifo_queue_level, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type navdata_video_stream
    let len;
    let data = new navdata_video_stream(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [drone_time]
    data.drone_time = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [tag]
    data.tag = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [size]
    data.size = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [quant]
    data.quant = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [frame_size]
    data.frame_size = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [frame_number]
    data.frame_number = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [atcmd_ref_seq]
    data.atcmd_ref_seq = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [atcmd_mean_ref_gap]
    data.atcmd_mean_ref_gap = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [atcmd_var_ref_gap]
    data.atcmd_var_ref_gap = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [atcmd_ref_quality]
    data.atcmd_ref_quality = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [desired_bitrate]
    data.desired_bitrate = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [data2]
    data.data2 = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [data3]
    data.data3 = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [data4]
    data.data4 = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [data5]
    data.data5 = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [fifo_queue_level]
    data.fifo_queue_level = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 61;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ardrone_autonomy/navdata_video_stream';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'caf560356d435eb563ce5b9e8278da00';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    float64 drone_time
    uint16 tag
    uint16 size
    uint8 quant
    uint32 frame_size
    uint32 frame_number
    uint32 atcmd_ref_seq
    uint32 atcmd_mean_ref_gap
    float32 atcmd_var_ref_gap
    uint32 atcmd_ref_quality
    uint32 desired_bitrate
    int32 data2
    int32 data3
    int32 data4
    int32 data5
    uint32 fifo_queue_level
    
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
    const resolved = new navdata_video_stream(null);
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

    if (msg.quant !== undefined) {
      resolved.quant = msg.quant;
    }
    else {
      resolved.quant = 0
    }

    if (msg.frame_size !== undefined) {
      resolved.frame_size = msg.frame_size;
    }
    else {
      resolved.frame_size = 0
    }

    if (msg.frame_number !== undefined) {
      resolved.frame_number = msg.frame_number;
    }
    else {
      resolved.frame_number = 0
    }

    if (msg.atcmd_ref_seq !== undefined) {
      resolved.atcmd_ref_seq = msg.atcmd_ref_seq;
    }
    else {
      resolved.atcmd_ref_seq = 0
    }

    if (msg.atcmd_mean_ref_gap !== undefined) {
      resolved.atcmd_mean_ref_gap = msg.atcmd_mean_ref_gap;
    }
    else {
      resolved.atcmd_mean_ref_gap = 0
    }

    if (msg.atcmd_var_ref_gap !== undefined) {
      resolved.atcmd_var_ref_gap = msg.atcmd_var_ref_gap;
    }
    else {
      resolved.atcmd_var_ref_gap = 0.0
    }

    if (msg.atcmd_ref_quality !== undefined) {
      resolved.atcmd_ref_quality = msg.atcmd_ref_quality;
    }
    else {
      resolved.atcmd_ref_quality = 0
    }

    if (msg.desired_bitrate !== undefined) {
      resolved.desired_bitrate = msg.desired_bitrate;
    }
    else {
      resolved.desired_bitrate = 0
    }

    if (msg.data2 !== undefined) {
      resolved.data2 = msg.data2;
    }
    else {
      resolved.data2 = 0
    }

    if (msg.data3 !== undefined) {
      resolved.data3 = msg.data3;
    }
    else {
      resolved.data3 = 0
    }

    if (msg.data4 !== undefined) {
      resolved.data4 = msg.data4;
    }
    else {
      resolved.data4 = 0
    }

    if (msg.data5 !== undefined) {
      resolved.data5 = msg.data5;
    }
    else {
      resolved.data5 = 0
    }

    if (msg.fifo_queue_level !== undefined) {
      resolved.fifo_queue_level = msg.fifo_queue_level;
    }
    else {
      resolved.fifo_queue_level = 0
    }

    return resolved;
    }
};

module.exports = navdata_video_stream;
