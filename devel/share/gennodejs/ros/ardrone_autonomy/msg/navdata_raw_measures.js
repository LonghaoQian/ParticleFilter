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

class navdata_raw_measures {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.drone_time = null;
      this.tag = null;
      this.size = null;
      this.raw_gyros = null;
      this.raw_gyros_110 = null;
      this.vbat_raw = null;
      this.us_debut_echo = null;
      this.us_fin_echo = null;
      this.us_association_echo = null;
      this.us_distance_echo = null;
      this.us_courbe_temps = null;
      this.us_courbe_valeur = null;
      this.us_courbe_ref = null;
      this.flag_echo_ini = null;
      this.nb_echo = null;
      this.sum_echo = null;
      this.alt_temp_raw = null;
      this.gradient = null;
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
      if (initObj.hasOwnProperty('raw_gyros')) {
        this.raw_gyros = initObj.raw_gyros
      }
      else {
        this.raw_gyros = [];
      }
      if (initObj.hasOwnProperty('raw_gyros_110')) {
        this.raw_gyros_110 = initObj.raw_gyros_110
      }
      else {
        this.raw_gyros_110 = [];
      }
      if (initObj.hasOwnProperty('vbat_raw')) {
        this.vbat_raw = initObj.vbat_raw
      }
      else {
        this.vbat_raw = 0;
      }
      if (initObj.hasOwnProperty('us_debut_echo')) {
        this.us_debut_echo = initObj.us_debut_echo
      }
      else {
        this.us_debut_echo = 0;
      }
      if (initObj.hasOwnProperty('us_fin_echo')) {
        this.us_fin_echo = initObj.us_fin_echo
      }
      else {
        this.us_fin_echo = 0;
      }
      if (initObj.hasOwnProperty('us_association_echo')) {
        this.us_association_echo = initObj.us_association_echo
      }
      else {
        this.us_association_echo = 0;
      }
      if (initObj.hasOwnProperty('us_distance_echo')) {
        this.us_distance_echo = initObj.us_distance_echo
      }
      else {
        this.us_distance_echo = 0;
      }
      if (initObj.hasOwnProperty('us_courbe_temps')) {
        this.us_courbe_temps = initObj.us_courbe_temps
      }
      else {
        this.us_courbe_temps = 0;
      }
      if (initObj.hasOwnProperty('us_courbe_valeur')) {
        this.us_courbe_valeur = initObj.us_courbe_valeur
      }
      else {
        this.us_courbe_valeur = 0;
      }
      if (initObj.hasOwnProperty('us_courbe_ref')) {
        this.us_courbe_ref = initObj.us_courbe_ref
      }
      else {
        this.us_courbe_ref = 0;
      }
      if (initObj.hasOwnProperty('flag_echo_ini')) {
        this.flag_echo_ini = initObj.flag_echo_ini
      }
      else {
        this.flag_echo_ini = 0;
      }
      if (initObj.hasOwnProperty('nb_echo')) {
        this.nb_echo = initObj.nb_echo
      }
      else {
        this.nb_echo = 0;
      }
      if (initObj.hasOwnProperty('sum_echo')) {
        this.sum_echo = initObj.sum_echo
      }
      else {
        this.sum_echo = 0;
      }
      if (initObj.hasOwnProperty('alt_temp_raw')) {
        this.alt_temp_raw = initObj.alt_temp_raw
      }
      else {
        this.alt_temp_raw = 0;
      }
      if (initObj.hasOwnProperty('gradient')) {
        this.gradient = initObj.gradient
      }
      else {
        this.gradient = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type navdata_raw_measures
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [drone_time]
    bufferOffset = _serializer.float64(obj.drone_time, buffer, bufferOffset);
    // Serialize message field [tag]
    bufferOffset = _serializer.uint16(obj.tag, buffer, bufferOffset);
    // Serialize message field [size]
    bufferOffset = _serializer.uint16(obj.size, buffer, bufferOffset);
    // Serialize message field [raw_gyros]
    bufferOffset = _arraySerializer.int16(obj.raw_gyros, buffer, bufferOffset, null);
    // Serialize message field [raw_gyros_110]
    bufferOffset = _arraySerializer.int16(obj.raw_gyros_110, buffer, bufferOffset, null);
    // Serialize message field [vbat_raw]
    bufferOffset = _serializer.uint32(obj.vbat_raw, buffer, bufferOffset);
    // Serialize message field [us_debut_echo]
    bufferOffset = _serializer.uint16(obj.us_debut_echo, buffer, bufferOffset);
    // Serialize message field [us_fin_echo]
    bufferOffset = _serializer.uint16(obj.us_fin_echo, buffer, bufferOffset);
    // Serialize message field [us_association_echo]
    bufferOffset = _serializer.uint16(obj.us_association_echo, buffer, bufferOffset);
    // Serialize message field [us_distance_echo]
    bufferOffset = _serializer.uint16(obj.us_distance_echo, buffer, bufferOffset);
    // Serialize message field [us_courbe_temps]
    bufferOffset = _serializer.uint16(obj.us_courbe_temps, buffer, bufferOffset);
    // Serialize message field [us_courbe_valeur]
    bufferOffset = _serializer.uint16(obj.us_courbe_valeur, buffer, bufferOffset);
    // Serialize message field [us_courbe_ref]
    bufferOffset = _serializer.uint16(obj.us_courbe_ref, buffer, bufferOffset);
    // Serialize message field [flag_echo_ini]
    bufferOffset = _serializer.uint16(obj.flag_echo_ini, buffer, bufferOffset);
    // Serialize message field [nb_echo]
    bufferOffset = _serializer.uint16(obj.nb_echo, buffer, bufferOffset);
    // Serialize message field [sum_echo]
    bufferOffset = _serializer.uint32(obj.sum_echo, buffer, bufferOffset);
    // Serialize message field [alt_temp_raw]
    bufferOffset = _serializer.int32(obj.alt_temp_raw, buffer, bufferOffset);
    // Serialize message field [gradient]
    bufferOffset = _serializer.int16(obj.gradient, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type navdata_raw_measures
    let len;
    let data = new navdata_raw_measures(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [drone_time]
    data.drone_time = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [tag]
    data.tag = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [size]
    data.size = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [raw_gyros]
    data.raw_gyros = _arrayDeserializer.int16(buffer, bufferOffset, null)
    // Deserialize message field [raw_gyros_110]
    data.raw_gyros_110 = _arrayDeserializer.int16(buffer, bufferOffset, null)
    // Deserialize message field [vbat_raw]
    data.vbat_raw = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [us_debut_echo]
    data.us_debut_echo = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [us_fin_echo]
    data.us_fin_echo = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [us_association_echo]
    data.us_association_echo = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [us_distance_echo]
    data.us_distance_echo = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [us_courbe_temps]
    data.us_courbe_temps = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [us_courbe_valeur]
    data.us_courbe_valeur = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [us_courbe_ref]
    data.us_courbe_ref = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [flag_echo_ini]
    data.flag_echo_ini = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [nb_echo]
    data.nb_echo = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [sum_echo]
    data.sum_echo = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [alt_temp_raw]
    data.alt_temp_raw = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [gradient]
    data.gradient = _deserializer.int16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 2 * object.raw_gyros.length;
    length += 2 * object.raw_gyros_110.length;
    return length + 52;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ardrone_autonomy/navdata_raw_measures';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4da7145c7478d1eb84be4d5fa4acd9ca';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    float64 drone_time
    uint16 tag
    uint16 size
    int16[] raw_gyros
    int16[] raw_gyros_110
    uint32 vbat_raw
    uint16 us_debut_echo
    uint16 us_fin_echo
    uint16 us_association_echo
    uint16 us_distance_echo
    uint16 us_courbe_temps
    uint16 us_courbe_valeur
    uint16 us_courbe_ref
    uint16 flag_echo_ini
    uint16 nb_echo
    uint32 sum_echo
    int32 alt_temp_raw
    int16 gradient
    
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
    const resolved = new navdata_raw_measures(null);
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

    if (msg.raw_gyros !== undefined) {
      resolved.raw_gyros = msg.raw_gyros;
    }
    else {
      resolved.raw_gyros = []
    }

    if (msg.raw_gyros_110 !== undefined) {
      resolved.raw_gyros_110 = msg.raw_gyros_110;
    }
    else {
      resolved.raw_gyros_110 = []
    }

    if (msg.vbat_raw !== undefined) {
      resolved.vbat_raw = msg.vbat_raw;
    }
    else {
      resolved.vbat_raw = 0
    }

    if (msg.us_debut_echo !== undefined) {
      resolved.us_debut_echo = msg.us_debut_echo;
    }
    else {
      resolved.us_debut_echo = 0
    }

    if (msg.us_fin_echo !== undefined) {
      resolved.us_fin_echo = msg.us_fin_echo;
    }
    else {
      resolved.us_fin_echo = 0
    }

    if (msg.us_association_echo !== undefined) {
      resolved.us_association_echo = msg.us_association_echo;
    }
    else {
      resolved.us_association_echo = 0
    }

    if (msg.us_distance_echo !== undefined) {
      resolved.us_distance_echo = msg.us_distance_echo;
    }
    else {
      resolved.us_distance_echo = 0
    }

    if (msg.us_courbe_temps !== undefined) {
      resolved.us_courbe_temps = msg.us_courbe_temps;
    }
    else {
      resolved.us_courbe_temps = 0
    }

    if (msg.us_courbe_valeur !== undefined) {
      resolved.us_courbe_valeur = msg.us_courbe_valeur;
    }
    else {
      resolved.us_courbe_valeur = 0
    }

    if (msg.us_courbe_ref !== undefined) {
      resolved.us_courbe_ref = msg.us_courbe_ref;
    }
    else {
      resolved.us_courbe_ref = 0
    }

    if (msg.flag_echo_ini !== undefined) {
      resolved.flag_echo_ini = msg.flag_echo_ini;
    }
    else {
      resolved.flag_echo_ini = 0
    }

    if (msg.nb_echo !== undefined) {
      resolved.nb_echo = msg.nb_echo;
    }
    else {
      resolved.nb_echo = 0
    }

    if (msg.sum_echo !== undefined) {
      resolved.sum_echo = msg.sum_echo;
    }
    else {
      resolved.sum_echo = 0
    }

    if (msg.alt_temp_raw !== undefined) {
      resolved.alt_temp_raw = msg.alt_temp_raw;
    }
    else {
      resolved.alt_temp_raw = 0
    }

    if (msg.gradient !== undefined) {
      resolved.gradient = msg.gradient;
    }
    else {
      resolved.gradient = 0
    }

    return resolved;
    }
};

module.exports = navdata_raw_measures;
