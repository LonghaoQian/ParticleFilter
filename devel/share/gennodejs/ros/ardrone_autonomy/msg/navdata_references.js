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

class navdata_references {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.drone_time = null;
      this.tag = null;
      this.size = null;
      this.ref_theta = null;
      this.ref_phi = null;
      this.ref_theta_I = null;
      this.ref_phi_I = null;
      this.ref_pitch = null;
      this.ref_roll = null;
      this.ref_yaw = null;
      this.ref_psi = null;
      this.vx_ref = null;
      this.vy_ref = null;
      this.theta_mod = null;
      this.phi_mod = null;
      this.k_v_x = null;
      this.k_v_y = null;
      this.k_mode = null;
      this.ui_time = null;
      this.ui_theta = null;
      this.ui_phi = null;
      this.ui_psi = null;
      this.ui_psi_accuracy = null;
      this.ui_seq = null;
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
      if (initObj.hasOwnProperty('ref_theta')) {
        this.ref_theta = initObj.ref_theta
      }
      else {
        this.ref_theta = 0;
      }
      if (initObj.hasOwnProperty('ref_phi')) {
        this.ref_phi = initObj.ref_phi
      }
      else {
        this.ref_phi = 0;
      }
      if (initObj.hasOwnProperty('ref_theta_I')) {
        this.ref_theta_I = initObj.ref_theta_I
      }
      else {
        this.ref_theta_I = 0;
      }
      if (initObj.hasOwnProperty('ref_phi_I')) {
        this.ref_phi_I = initObj.ref_phi_I
      }
      else {
        this.ref_phi_I = 0;
      }
      if (initObj.hasOwnProperty('ref_pitch')) {
        this.ref_pitch = initObj.ref_pitch
      }
      else {
        this.ref_pitch = 0;
      }
      if (initObj.hasOwnProperty('ref_roll')) {
        this.ref_roll = initObj.ref_roll
      }
      else {
        this.ref_roll = 0;
      }
      if (initObj.hasOwnProperty('ref_yaw')) {
        this.ref_yaw = initObj.ref_yaw
      }
      else {
        this.ref_yaw = 0;
      }
      if (initObj.hasOwnProperty('ref_psi')) {
        this.ref_psi = initObj.ref_psi
      }
      else {
        this.ref_psi = 0;
      }
      if (initObj.hasOwnProperty('vx_ref')) {
        this.vx_ref = initObj.vx_ref
      }
      else {
        this.vx_ref = 0.0;
      }
      if (initObj.hasOwnProperty('vy_ref')) {
        this.vy_ref = initObj.vy_ref
      }
      else {
        this.vy_ref = 0.0;
      }
      if (initObj.hasOwnProperty('theta_mod')) {
        this.theta_mod = initObj.theta_mod
      }
      else {
        this.theta_mod = 0.0;
      }
      if (initObj.hasOwnProperty('phi_mod')) {
        this.phi_mod = initObj.phi_mod
      }
      else {
        this.phi_mod = 0.0;
      }
      if (initObj.hasOwnProperty('k_v_x')) {
        this.k_v_x = initObj.k_v_x
      }
      else {
        this.k_v_x = 0.0;
      }
      if (initObj.hasOwnProperty('k_v_y')) {
        this.k_v_y = initObj.k_v_y
      }
      else {
        this.k_v_y = 0.0;
      }
      if (initObj.hasOwnProperty('k_mode')) {
        this.k_mode = initObj.k_mode
      }
      else {
        this.k_mode = 0;
      }
      if (initObj.hasOwnProperty('ui_time')) {
        this.ui_time = initObj.ui_time
      }
      else {
        this.ui_time = 0.0;
      }
      if (initObj.hasOwnProperty('ui_theta')) {
        this.ui_theta = initObj.ui_theta
      }
      else {
        this.ui_theta = 0.0;
      }
      if (initObj.hasOwnProperty('ui_phi')) {
        this.ui_phi = initObj.ui_phi
      }
      else {
        this.ui_phi = 0.0;
      }
      if (initObj.hasOwnProperty('ui_psi')) {
        this.ui_psi = initObj.ui_psi
      }
      else {
        this.ui_psi = 0.0;
      }
      if (initObj.hasOwnProperty('ui_psi_accuracy')) {
        this.ui_psi_accuracy = initObj.ui_psi_accuracy
      }
      else {
        this.ui_psi_accuracy = 0.0;
      }
      if (initObj.hasOwnProperty('ui_seq')) {
        this.ui_seq = initObj.ui_seq
      }
      else {
        this.ui_seq = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type navdata_references
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [drone_time]
    bufferOffset = _serializer.float64(obj.drone_time, buffer, bufferOffset);
    // Serialize message field [tag]
    bufferOffset = _serializer.uint16(obj.tag, buffer, bufferOffset);
    // Serialize message field [size]
    bufferOffset = _serializer.uint16(obj.size, buffer, bufferOffset);
    // Serialize message field [ref_theta]
    bufferOffset = _serializer.int32(obj.ref_theta, buffer, bufferOffset);
    // Serialize message field [ref_phi]
    bufferOffset = _serializer.int32(obj.ref_phi, buffer, bufferOffset);
    // Serialize message field [ref_theta_I]
    bufferOffset = _serializer.int32(obj.ref_theta_I, buffer, bufferOffset);
    // Serialize message field [ref_phi_I]
    bufferOffset = _serializer.int32(obj.ref_phi_I, buffer, bufferOffset);
    // Serialize message field [ref_pitch]
    bufferOffset = _serializer.int32(obj.ref_pitch, buffer, bufferOffset);
    // Serialize message field [ref_roll]
    bufferOffset = _serializer.int32(obj.ref_roll, buffer, bufferOffset);
    // Serialize message field [ref_yaw]
    bufferOffset = _serializer.int32(obj.ref_yaw, buffer, bufferOffset);
    // Serialize message field [ref_psi]
    bufferOffset = _serializer.int32(obj.ref_psi, buffer, bufferOffset);
    // Serialize message field [vx_ref]
    bufferOffset = _serializer.float32(obj.vx_ref, buffer, bufferOffset);
    // Serialize message field [vy_ref]
    bufferOffset = _serializer.float32(obj.vy_ref, buffer, bufferOffset);
    // Serialize message field [theta_mod]
    bufferOffset = _serializer.float32(obj.theta_mod, buffer, bufferOffset);
    // Serialize message field [phi_mod]
    bufferOffset = _serializer.float32(obj.phi_mod, buffer, bufferOffset);
    // Serialize message field [k_v_x]
    bufferOffset = _serializer.float32(obj.k_v_x, buffer, bufferOffset);
    // Serialize message field [k_v_y]
    bufferOffset = _serializer.float32(obj.k_v_y, buffer, bufferOffset);
    // Serialize message field [k_mode]
    bufferOffset = _serializer.uint32(obj.k_mode, buffer, bufferOffset);
    // Serialize message field [ui_time]
    bufferOffset = _serializer.float32(obj.ui_time, buffer, bufferOffset);
    // Serialize message field [ui_theta]
    bufferOffset = _serializer.float32(obj.ui_theta, buffer, bufferOffset);
    // Serialize message field [ui_phi]
    bufferOffset = _serializer.float32(obj.ui_phi, buffer, bufferOffset);
    // Serialize message field [ui_psi]
    bufferOffset = _serializer.float32(obj.ui_psi, buffer, bufferOffset);
    // Serialize message field [ui_psi_accuracy]
    bufferOffset = _serializer.float32(obj.ui_psi_accuracy, buffer, bufferOffset);
    // Serialize message field [ui_seq]
    bufferOffset = _serializer.int32(obj.ui_seq, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type navdata_references
    let len;
    let data = new navdata_references(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [drone_time]
    data.drone_time = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [tag]
    data.tag = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [size]
    data.size = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [ref_theta]
    data.ref_theta = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [ref_phi]
    data.ref_phi = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [ref_theta_I]
    data.ref_theta_I = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [ref_phi_I]
    data.ref_phi_I = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [ref_pitch]
    data.ref_pitch = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [ref_roll]
    data.ref_roll = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [ref_yaw]
    data.ref_yaw = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [ref_psi]
    data.ref_psi = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [vx_ref]
    data.vx_ref = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [vy_ref]
    data.vy_ref = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [theta_mod]
    data.theta_mod = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [phi_mod]
    data.phi_mod = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [k_v_x]
    data.k_v_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [k_v_y]
    data.k_v_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [k_mode]
    data.k_mode = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [ui_time]
    data.ui_time = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [ui_theta]
    data.ui_theta = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [ui_phi]
    data.ui_phi = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [ui_psi]
    data.ui_psi = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [ui_psi_accuracy]
    data.ui_psi_accuracy = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [ui_seq]
    data.ui_seq = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 96;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ardrone_autonomy/navdata_references';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '263b844b053f4a098c75c1c26a452911';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    float64 drone_time
    uint16 tag
    uint16 size
    int32 ref_theta
    int32 ref_phi
    int32 ref_theta_I
    int32 ref_phi_I
    int32 ref_pitch
    int32 ref_roll
    int32 ref_yaw
    int32 ref_psi
    float32 vx_ref
    float32 vy_ref
    float32 theta_mod
    float32 phi_mod
    float32 k_v_x
    float32 k_v_y
    uint32 k_mode
    float32 ui_time
    float32 ui_theta
    float32 ui_phi
    float32 ui_psi
    float32 ui_psi_accuracy
    int32 ui_seq
    
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
    const resolved = new navdata_references(null);
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

    if (msg.ref_theta !== undefined) {
      resolved.ref_theta = msg.ref_theta;
    }
    else {
      resolved.ref_theta = 0
    }

    if (msg.ref_phi !== undefined) {
      resolved.ref_phi = msg.ref_phi;
    }
    else {
      resolved.ref_phi = 0
    }

    if (msg.ref_theta_I !== undefined) {
      resolved.ref_theta_I = msg.ref_theta_I;
    }
    else {
      resolved.ref_theta_I = 0
    }

    if (msg.ref_phi_I !== undefined) {
      resolved.ref_phi_I = msg.ref_phi_I;
    }
    else {
      resolved.ref_phi_I = 0
    }

    if (msg.ref_pitch !== undefined) {
      resolved.ref_pitch = msg.ref_pitch;
    }
    else {
      resolved.ref_pitch = 0
    }

    if (msg.ref_roll !== undefined) {
      resolved.ref_roll = msg.ref_roll;
    }
    else {
      resolved.ref_roll = 0
    }

    if (msg.ref_yaw !== undefined) {
      resolved.ref_yaw = msg.ref_yaw;
    }
    else {
      resolved.ref_yaw = 0
    }

    if (msg.ref_psi !== undefined) {
      resolved.ref_psi = msg.ref_psi;
    }
    else {
      resolved.ref_psi = 0
    }

    if (msg.vx_ref !== undefined) {
      resolved.vx_ref = msg.vx_ref;
    }
    else {
      resolved.vx_ref = 0.0
    }

    if (msg.vy_ref !== undefined) {
      resolved.vy_ref = msg.vy_ref;
    }
    else {
      resolved.vy_ref = 0.0
    }

    if (msg.theta_mod !== undefined) {
      resolved.theta_mod = msg.theta_mod;
    }
    else {
      resolved.theta_mod = 0.0
    }

    if (msg.phi_mod !== undefined) {
      resolved.phi_mod = msg.phi_mod;
    }
    else {
      resolved.phi_mod = 0.0
    }

    if (msg.k_v_x !== undefined) {
      resolved.k_v_x = msg.k_v_x;
    }
    else {
      resolved.k_v_x = 0.0
    }

    if (msg.k_v_y !== undefined) {
      resolved.k_v_y = msg.k_v_y;
    }
    else {
      resolved.k_v_y = 0.0
    }

    if (msg.k_mode !== undefined) {
      resolved.k_mode = msg.k_mode;
    }
    else {
      resolved.k_mode = 0
    }

    if (msg.ui_time !== undefined) {
      resolved.ui_time = msg.ui_time;
    }
    else {
      resolved.ui_time = 0.0
    }

    if (msg.ui_theta !== undefined) {
      resolved.ui_theta = msg.ui_theta;
    }
    else {
      resolved.ui_theta = 0.0
    }

    if (msg.ui_phi !== undefined) {
      resolved.ui_phi = msg.ui_phi;
    }
    else {
      resolved.ui_phi = 0.0
    }

    if (msg.ui_psi !== undefined) {
      resolved.ui_psi = msg.ui_psi;
    }
    else {
      resolved.ui_psi = 0.0
    }

    if (msg.ui_psi_accuracy !== undefined) {
      resolved.ui_psi_accuracy = msg.ui_psi_accuracy;
    }
    else {
      resolved.ui_psi_accuracy = 0.0
    }

    if (msg.ui_seq !== undefined) {
      resolved.ui_seq = msg.ui_seq;
    }
    else {
      resolved.ui_seq = 0
    }

    return resolved;
    }
};

module.exports = navdata_references;
