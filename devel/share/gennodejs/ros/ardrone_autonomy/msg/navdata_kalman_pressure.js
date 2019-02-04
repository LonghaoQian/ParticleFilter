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

class navdata_kalman_pressure {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.drone_time = null;
      this.tag = null;
      this.size = null;
      this.offset_pressure = null;
      this.est_z = null;
      this.est_zdot = null;
      this.est_bias_PWM = null;
      this.est_biais_pression = null;
      this.offset_US = null;
      this.prediction_US = null;
      this.cov_alt = null;
      this.cov_PWM = null;
      this.cov_vitesse = null;
      this.bool_effet_sol = null;
      this.somme_inno = null;
      this.flag_rejet_US = null;
      this.u_multisinus = null;
      this.gaz_altitude = null;
      this.Flag_multisinus = null;
      this.Flag_multisinus_debut = null;
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
      if (initObj.hasOwnProperty('offset_pressure')) {
        this.offset_pressure = initObj.offset_pressure
      }
      else {
        this.offset_pressure = 0.0;
      }
      if (initObj.hasOwnProperty('est_z')) {
        this.est_z = initObj.est_z
      }
      else {
        this.est_z = 0.0;
      }
      if (initObj.hasOwnProperty('est_zdot')) {
        this.est_zdot = initObj.est_zdot
      }
      else {
        this.est_zdot = 0.0;
      }
      if (initObj.hasOwnProperty('est_bias_PWM')) {
        this.est_bias_PWM = initObj.est_bias_PWM
      }
      else {
        this.est_bias_PWM = 0.0;
      }
      if (initObj.hasOwnProperty('est_biais_pression')) {
        this.est_biais_pression = initObj.est_biais_pression
      }
      else {
        this.est_biais_pression = 0.0;
      }
      if (initObj.hasOwnProperty('offset_US')) {
        this.offset_US = initObj.offset_US
      }
      else {
        this.offset_US = 0.0;
      }
      if (initObj.hasOwnProperty('prediction_US')) {
        this.prediction_US = initObj.prediction_US
      }
      else {
        this.prediction_US = 0.0;
      }
      if (initObj.hasOwnProperty('cov_alt')) {
        this.cov_alt = initObj.cov_alt
      }
      else {
        this.cov_alt = 0.0;
      }
      if (initObj.hasOwnProperty('cov_PWM')) {
        this.cov_PWM = initObj.cov_PWM
      }
      else {
        this.cov_PWM = 0.0;
      }
      if (initObj.hasOwnProperty('cov_vitesse')) {
        this.cov_vitesse = initObj.cov_vitesse
      }
      else {
        this.cov_vitesse = 0.0;
      }
      if (initObj.hasOwnProperty('bool_effet_sol')) {
        this.bool_effet_sol = initObj.bool_effet_sol
      }
      else {
        this.bool_effet_sol = 0;
      }
      if (initObj.hasOwnProperty('somme_inno')) {
        this.somme_inno = initObj.somme_inno
      }
      else {
        this.somme_inno = 0.0;
      }
      if (initObj.hasOwnProperty('flag_rejet_US')) {
        this.flag_rejet_US = initObj.flag_rejet_US
      }
      else {
        this.flag_rejet_US = 0;
      }
      if (initObj.hasOwnProperty('u_multisinus')) {
        this.u_multisinus = initObj.u_multisinus
      }
      else {
        this.u_multisinus = 0.0;
      }
      if (initObj.hasOwnProperty('gaz_altitude')) {
        this.gaz_altitude = initObj.gaz_altitude
      }
      else {
        this.gaz_altitude = 0.0;
      }
      if (initObj.hasOwnProperty('Flag_multisinus')) {
        this.Flag_multisinus = initObj.Flag_multisinus
      }
      else {
        this.Flag_multisinus = 0;
      }
      if (initObj.hasOwnProperty('Flag_multisinus_debut')) {
        this.Flag_multisinus_debut = initObj.Flag_multisinus_debut
      }
      else {
        this.Flag_multisinus_debut = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type navdata_kalman_pressure
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [drone_time]
    bufferOffset = _serializer.float64(obj.drone_time, buffer, bufferOffset);
    // Serialize message field [tag]
    bufferOffset = _serializer.uint16(obj.tag, buffer, bufferOffset);
    // Serialize message field [size]
    bufferOffset = _serializer.uint16(obj.size, buffer, bufferOffset);
    // Serialize message field [offset_pressure]
    bufferOffset = _serializer.float32(obj.offset_pressure, buffer, bufferOffset);
    // Serialize message field [est_z]
    bufferOffset = _serializer.float32(obj.est_z, buffer, bufferOffset);
    // Serialize message field [est_zdot]
    bufferOffset = _serializer.float32(obj.est_zdot, buffer, bufferOffset);
    // Serialize message field [est_bias_PWM]
    bufferOffset = _serializer.float32(obj.est_bias_PWM, buffer, bufferOffset);
    // Serialize message field [est_biais_pression]
    bufferOffset = _serializer.float32(obj.est_biais_pression, buffer, bufferOffset);
    // Serialize message field [offset_US]
    bufferOffset = _serializer.float32(obj.offset_US, buffer, bufferOffset);
    // Serialize message field [prediction_US]
    bufferOffset = _serializer.float32(obj.prediction_US, buffer, bufferOffset);
    // Serialize message field [cov_alt]
    bufferOffset = _serializer.float32(obj.cov_alt, buffer, bufferOffset);
    // Serialize message field [cov_PWM]
    bufferOffset = _serializer.float32(obj.cov_PWM, buffer, bufferOffset);
    // Serialize message field [cov_vitesse]
    bufferOffset = _serializer.float32(obj.cov_vitesse, buffer, bufferOffset);
    // Serialize message field [bool_effet_sol]
    bufferOffset = _serializer.int32(obj.bool_effet_sol, buffer, bufferOffset);
    // Serialize message field [somme_inno]
    bufferOffset = _serializer.float32(obj.somme_inno, buffer, bufferOffset);
    // Serialize message field [flag_rejet_US]
    bufferOffset = _serializer.int32(obj.flag_rejet_US, buffer, bufferOffset);
    // Serialize message field [u_multisinus]
    bufferOffset = _serializer.float32(obj.u_multisinus, buffer, bufferOffset);
    // Serialize message field [gaz_altitude]
    bufferOffset = _serializer.float32(obj.gaz_altitude, buffer, bufferOffset);
    // Serialize message field [Flag_multisinus]
    bufferOffset = _serializer.int32(obj.Flag_multisinus, buffer, bufferOffset);
    // Serialize message field [Flag_multisinus_debut]
    bufferOffset = _serializer.int32(obj.Flag_multisinus_debut, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type navdata_kalman_pressure
    let len;
    let data = new navdata_kalman_pressure(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [drone_time]
    data.drone_time = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [tag]
    data.tag = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [size]
    data.size = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [offset_pressure]
    data.offset_pressure = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [est_z]
    data.est_z = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [est_zdot]
    data.est_zdot = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [est_bias_PWM]
    data.est_bias_PWM = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [est_biais_pression]
    data.est_biais_pression = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [offset_US]
    data.offset_US = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [prediction_US]
    data.prediction_US = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [cov_alt]
    data.cov_alt = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [cov_PWM]
    data.cov_PWM = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [cov_vitesse]
    data.cov_vitesse = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [bool_effet_sol]
    data.bool_effet_sol = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [somme_inno]
    data.somme_inno = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [flag_rejet_US]
    data.flag_rejet_US = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [u_multisinus]
    data.u_multisinus = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [gaz_altitude]
    data.gaz_altitude = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [Flag_multisinus]
    data.Flag_multisinus = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [Flag_multisinus_debut]
    data.Flag_multisinus_debut = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 80;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ardrone_autonomy/navdata_kalman_pressure';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '70734b6caff0fb7ea6fc88ffeea5cde5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    float64 drone_time
    uint16 tag
    uint16 size
    float32 offset_pressure
    float32 est_z
    float32 est_zdot
    float32 est_bias_PWM
    float32 est_biais_pression
    float32 offset_US
    float32 prediction_US
    float32 cov_alt
    float32 cov_PWM
    float32 cov_vitesse
    int32 bool_effet_sol
    float32 somme_inno
    int32 flag_rejet_US
    float32 u_multisinus
    float32 gaz_altitude
    int32 Flag_multisinus
    int32 Flag_multisinus_debut
    
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
    const resolved = new navdata_kalman_pressure(null);
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

    if (msg.offset_pressure !== undefined) {
      resolved.offset_pressure = msg.offset_pressure;
    }
    else {
      resolved.offset_pressure = 0.0
    }

    if (msg.est_z !== undefined) {
      resolved.est_z = msg.est_z;
    }
    else {
      resolved.est_z = 0.0
    }

    if (msg.est_zdot !== undefined) {
      resolved.est_zdot = msg.est_zdot;
    }
    else {
      resolved.est_zdot = 0.0
    }

    if (msg.est_bias_PWM !== undefined) {
      resolved.est_bias_PWM = msg.est_bias_PWM;
    }
    else {
      resolved.est_bias_PWM = 0.0
    }

    if (msg.est_biais_pression !== undefined) {
      resolved.est_biais_pression = msg.est_biais_pression;
    }
    else {
      resolved.est_biais_pression = 0.0
    }

    if (msg.offset_US !== undefined) {
      resolved.offset_US = msg.offset_US;
    }
    else {
      resolved.offset_US = 0.0
    }

    if (msg.prediction_US !== undefined) {
      resolved.prediction_US = msg.prediction_US;
    }
    else {
      resolved.prediction_US = 0.0
    }

    if (msg.cov_alt !== undefined) {
      resolved.cov_alt = msg.cov_alt;
    }
    else {
      resolved.cov_alt = 0.0
    }

    if (msg.cov_PWM !== undefined) {
      resolved.cov_PWM = msg.cov_PWM;
    }
    else {
      resolved.cov_PWM = 0.0
    }

    if (msg.cov_vitesse !== undefined) {
      resolved.cov_vitesse = msg.cov_vitesse;
    }
    else {
      resolved.cov_vitesse = 0.0
    }

    if (msg.bool_effet_sol !== undefined) {
      resolved.bool_effet_sol = msg.bool_effet_sol;
    }
    else {
      resolved.bool_effet_sol = 0
    }

    if (msg.somme_inno !== undefined) {
      resolved.somme_inno = msg.somme_inno;
    }
    else {
      resolved.somme_inno = 0.0
    }

    if (msg.flag_rejet_US !== undefined) {
      resolved.flag_rejet_US = msg.flag_rejet_US;
    }
    else {
      resolved.flag_rejet_US = 0
    }

    if (msg.u_multisinus !== undefined) {
      resolved.u_multisinus = msg.u_multisinus;
    }
    else {
      resolved.u_multisinus = 0.0
    }

    if (msg.gaz_altitude !== undefined) {
      resolved.gaz_altitude = msg.gaz_altitude;
    }
    else {
      resolved.gaz_altitude = 0.0
    }

    if (msg.Flag_multisinus !== undefined) {
      resolved.Flag_multisinus = msg.Flag_multisinus;
    }
    else {
      resolved.Flag_multisinus = 0
    }

    if (msg.Flag_multisinus_debut !== undefined) {
      resolved.Flag_multisinus_debut = msg.Flag_multisinus_debut;
    }
    else {
      resolved.Flag_multisinus_debut = 0
    }

    return resolved;
    }
};

module.exports = navdata_kalman_pressure;
