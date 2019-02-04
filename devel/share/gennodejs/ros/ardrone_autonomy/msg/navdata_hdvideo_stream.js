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

class navdata_hdvideo_stream {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.drone_time = null;
      this.tag = null;
      this.size = null;
      this.hdvideo_state = null;
      this.storage_fifo_nb_packets = null;
      this.storage_fifo_size = null;
      this.usbkey_size = null;
      this.usbkey_freespace = null;
      this.frame_number = null;
      this.usbkey_remaining_time = null;
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
      if (initObj.hasOwnProperty('hdvideo_state')) {
        this.hdvideo_state = initObj.hdvideo_state
      }
      else {
        this.hdvideo_state = 0;
      }
      if (initObj.hasOwnProperty('storage_fifo_nb_packets')) {
        this.storage_fifo_nb_packets = initObj.storage_fifo_nb_packets
      }
      else {
        this.storage_fifo_nb_packets = 0;
      }
      if (initObj.hasOwnProperty('storage_fifo_size')) {
        this.storage_fifo_size = initObj.storage_fifo_size
      }
      else {
        this.storage_fifo_size = 0;
      }
      if (initObj.hasOwnProperty('usbkey_size')) {
        this.usbkey_size = initObj.usbkey_size
      }
      else {
        this.usbkey_size = 0;
      }
      if (initObj.hasOwnProperty('usbkey_freespace')) {
        this.usbkey_freespace = initObj.usbkey_freespace
      }
      else {
        this.usbkey_freespace = 0;
      }
      if (initObj.hasOwnProperty('frame_number')) {
        this.frame_number = initObj.frame_number
      }
      else {
        this.frame_number = 0;
      }
      if (initObj.hasOwnProperty('usbkey_remaining_time')) {
        this.usbkey_remaining_time = initObj.usbkey_remaining_time
      }
      else {
        this.usbkey_remaining_time = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type navdata_hdvideo_stream
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [drone_time]
    bufferOffset = _serializer.float64(obj.drone_time, buffer, bufferOffset);
    // Serialize message field [tag]
    bufferOffset = _serializer.uint16(obj.tag, buffer, bufferOffset);
    // Serialize message field [size]
    bufferOffset = _serializer.uint16(obj.size, buffer, bufferOffset);
    // Serialize message field [hdvideo_state]
    bufferOffset = _serializer.uint32(obj.hdvideo_state, buffer, bufferOffset);
    // Serialize message field [storage_fifo_nb_packets]
    bufferOffset = _serializer.uint32(obj.storage_fifo_nb_packets, buffer, bufferOffset);
    // Serialize message field [storage_fifo_size]
    bufferOffset = _serializer.uint32(obj.storage_fifo_size, buffer, bufferOffset);
    // Serialize message field [usbkey_size]
    bufferOffset = _serializer.uint32(obj.usbkey_size, buffer, bufferOffset);
    // Serialize message field [usbkey_freespace]
    bufferOffset = _serializer.uint32(obj.usbkey_freespace, buffer, bufferOffset);
    // Serialize message field [frame_number]
    bufferOffset = _serializer.uint32(obj.frame_number, buffer, bufferOffset);
    // Serialize message field [usbkey_remaining_time]
    bufferOffset = _serializer.uint32(obj.usbkey_remaining_time, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type navdata_hdvideo_stream
    let len;
    let data = new navdata_hdvideo_stream(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [drone_time]
    data.drone_time = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [tag]
    data.tag = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [size]
    data.size = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [hdvideo_state]
    data.hdvideo_state = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [storage_fifo_nb_packets]
    data.storage_fifo_nb_packets = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [storage_fifo_size]
    data.storage_fifo_size = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [usbkey_size]
    data.usbkey_size = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [usbkey_freespace]
    data.usbkey_freespace = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [frame_number]
    data.frame_number = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [usbkey_remaining_time]
    data.usbkey_remaining_time = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 40;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ardrone_autonomy/navdata_hdvideo_stream';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1ba321578916df95f899ca2f5348f234';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    float64 drone_time
    uint16 tag
    uint16 size
    uint32 hdvideo_state
    uint32 storage_fifo_nb_packets
    uint32 storage_fifo_size
    uint32 usbkey_size
    uint32 usbkey_freespace
    uint32 frame_number
    uint32 usbkey_remaining_time
    
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
    const resolved = new navdata_hdvideo_stream(null);
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

    if (msg.hdvideo_state !== undefined) {
      resolved.hdvideo_state = msg.hdvideo_state;
    }
    else {
      resolved.hdvideo_state = 0
    }

    if (msg.storage_fifo_nb_packets !== undefined) {
      resolved.storage_fifo_nb_packets = msg.storage_fifo_nb_packets;
    }
    else {
      resolved.storage_fifo_nb_packets = 0
    }

    if (msg.storage_fifo_size !== undefined) {
      resolved.storage_fifo_size = msg.storage_fifo_size;
    }
    else {
      resolved.storage_fifo_size = 0
    }

    if (msg.usbkey_size !== undefined) {
      resolved.usbkey_size = msg.usbkey_size;
    }
    else {
      resolved.usbkey_size = 0
    }

    if (msg.usbkey_freespace !== undefined) {
      resolved.usbkey_freespace = msg.usbkey_freespace;
    }
    else {
      resolved.usbkey_freespace = 0
    }

    if (msg.frame_number !== undefined) {
      resolved.frame_number = msg.frame_number;
    }
    else {
      resolved.frame_number = 0
    }

    if (msg.usbkey_remaining_time !== undefined) {
      resolved.usbkey_remaining_time = msg.usbkey_remaining_time;
    }
    else {
      resolved.usbkey_remaining_time = 0
    }

    return resolved;
    }
};

module.exports = navdata_hdvideo_stream;
