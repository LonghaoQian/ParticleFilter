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

class Navdata {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.batteryPercent = null;
      this.state = null;
      this.magX = null;
      this.magY = null;
      this.magZ = null;
      this.pressure = null;
      this.temp = null;
      this.wind_speed = null;
      this.wind_angle = null;
      this.wind_comp_angle = null;
      this.rotX = null;
      this.rotY = null;
      this.rotZ = null;
      this.altd = null;
      this.vx = null;
      this.vy = null;
      this.vz = null;
      this.ax = null;
      this.ay = null;
      this.az = null;
      this.motor1 = null;
      this.motor2 = null;
      this.motor3 = null;
      this.motor4 = null;
      this.tags_count = null;
      this.tags_type = null;
      this.tags_xc = null;
      this.tags_yc = null;
      this.tags_width = null;
      this.tags_height = null;
      this.tags_orientation = null;
      this.tags_distance = null;
      this.tm = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('batteryPercent')) {
        this.batteryPercent = initObj.batteryPercent
      }
      else {
        this.batteryPercent = 0.0;
      }
      if (initObj.hasOwnProperty('state')) {
        this.state = initObj.state
      }
      else {
        this.state = 0;
      }
      if (initObj.hasOwnProperty('magX')) {
        this.magX = initObj.magX
      }
      else {
        this.magX = 0;
      }
      if (initObj.hasOwnProperty('magY')) {
        this.magY = initObj.magY
      }
      else {
        this.magY = 0;
      }
      if (initObj.hasOwnProperty('magZ')) {
        this.magZ = initObj.magZ
      }
      else {
        this.magZ = 0;
      }
      if (initObj.hasOwnProperty('pressure')) {
        this.pressure = initObj.pressure
      }
      else {
        this.pressure = 0;
      }
      if (initObj.hasOwnProperty('temp')) {
        this.temp = initObj.temp
      }
      else {
        this.temp = 0;
      }
      if (initObj.hasOwnProperty('wind_speed')) {
        this.wind_speed = initObj.wind_speed
      }
      else {
        this.wind_speed = 0.0;
      }
      if (initObj.hasOwnProperty('wind_angle')) {
        this.wind_angle = initObj.wind_angle
      }
      else {
        this.wind_angle = 0.0;
      }
      if (initObj.hasOwnProperty('wind_comp_angle')) {
        this.wind_comp_angle = initObj.wind_comp_angle
      }
      else {
        this.wind_comp_angle = 0.0;
      }
      if (initObj.hasOwnProperty('rotX')) {
        this.rotX = initObj.rotX
      }
      else {
        this.rotX = 0.0;
      }
      if (initObj.hasOwnProperty('rotY')) {
        this.rotY = initObj.rotY
      }
      else {
        this.rotY = 0.0;
      }
      if (initObj.hasOwnProperty('rotZ')) {
        this.rotZ = initObj.rotZ
      }
      else {
        this.rotZ = 0.0;
      }
      if (initObj.hasOwnProperty('altd')) {
        this.altd = initObj.altd
      }
      else {
        this.altd = 0;
      }
      if (initObj.hasOwnProperty('vx')) {
        this.vx = initObj.vx
      }
      else {
        this.vx = 0.0;
      }
      if (initObj.hasOwnProperty('vy')) {
        this.vy = initObj.vy
      }
      else {
        this.vy = 0.0;
      }
      if (initObj.hasOwnProperty('vz')) {
        this.vz = initObj.vz
      }
      else {
        this.vz = 0.0;
      }
      if (initObj.hasOwnProperty('ax')) {
        this.ax = initObj.ax
      }
      else {
        this.ax = 0.0;
      }
      if (initObj.hasOwnProperty('ay')) {
        this.ay = initObj.ay
      }
      else {
        this.ay = 0.0;
      }
      if (initObj.hasOwnProperty('az')) {
        this.az = initObj.az
      }
      else {
        this.az = 0.0;
      }
      if (initObj.hasOwnProperty('motor1')) {
        this.motor1 = initObj.motor1
      }
      else {
        this.motor1 = 0;
      }
      if (initObj.hasOwnProperty('motor2')) {
        this.motor2 = initObj.motor2
      }
      else {
        this.motor2 = 0;
      }
      if (initObj.hasOwnProperty('motor3')) {
        this.motor3 = initObj.motor3
      }
      else {
        this.motor3 = 0;
      }
      if (initObj.hasOwnProperty('motor4')) {
        this.motor4 = initObj.motor4
      }
      else {
        this.motor4 = 0;
      }
      if (initObj.hasOwnProperty('tags_count')) {
        this.tags_count = initObj.tags_count
      }
      else {
        this.tags_count = 0;
      }
      if (initObj.hasOwnProperty('tags_type')) {
        this.tags_type = initObj.tags_type
      }
      else {
        this.tags_type = [];
      }
      if (initObj.hasOwnProperty('tags_xc')) {
        this.tags_xc = initObj.tags_xc
      }
      else {
        this.tags_xc = [];
      }
      if (initObj.hasOwnProperty('tags_yc')) {
        this.tags_yc = initObj.tags_yc
      }
      else {
        this.tags_yc = [];
      }
      if (initObj.hasOwnProperty('tags_width')) {
        this.tags_width = initObj.tags_width
      }
      else {
        this.tags_width = [];
      }
      if (initObj.hasOwnProperty('tags_height')) {
        this.tags_height = initObj.tags_height
      }
      else {
        this.tags_height = [];
      }
      if (initObj.hasOwnProperty('tags_orientation')) {
        this.tags_orientation = initObj.tags_orientation
      }
      else {
        this.tags_orientation = [];
      }
      if (initObj.hasOwnProperty('tags_distance')) {
        this.tags_distance = initObj.tags_distance
      }
      else {
        this.tags_distance = [];
      }
      if (initObj.hasOwnProperty('tm')) {
        this.tm = initObj.tm
      }
      else {
        this.tm = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Navdata
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [batteryPercent]
    bufferOffset = _serializer.float32(obj.batteryPercent, buffer, bufferOffset);
    // Serialize message field [state]
    bufferOffset = _serializer.uint32(obj.state, buffer, bufferOffset);
    // Serialize message field [magX]
    bufferOffset = _serializer.int32(obj.magX, buffer, bufferOffset);
    // Serialize message field [magY]
    bufferOffset = _serializer.int32(obj.magY, buffer, bufferOffset);
    // Serialize message field [magZ]
    bufferOffset = _serializer.int32(obj.magZ, buffer, bufferOffset);
    // Serialize message field [pressure]
    bufferOffset = _serializer.int32(obj.pressure, buffer, bufferOffset);
    // Serialize message field [temp]
    bufferOffset = _serializer.int32(obj.temp, buffer, bufferOffset);
    // Serialize message field [wind_speed]
    bufferOffset = _serializer.float32(obj.wind_speed, buffer, bufferOffset);
    // Serialize message field [wind_angle]
    bufferOffset = _serializer.float32(obj.wind_angle, buffer, bufferOffset);
    // Serialize message field [wind_comp_angle]
    bufferOffset = _serializer.float32(obj.wind_comp_angle, buffer, bufferOffset);
    // Serialize message field [rotX]
    bufferOffset = _serializer.float32(obj.rotX, buffer, bufferOffset);
    // Serialize message field [rotY]
    bufferOffset = _serializer.float32(obj.rotY, buffer, bufferOffset);
    // Serialize message field [rotZ]
    bufferOffset = _serializer.float32(obj.rotZ, buffer, bufferOffset);
    // Serialize message field [altd]
    bufferOffset = _serializer.int32(obj.altd, buffer, bufferOffset);
    // Serialize message field [vx]
    bufferOffset = _serializer.float32(obj.vx, buffer, bufferOffset);
    // Serialize message field [vy]
    bufferOffset = _serializer.float32(obj.vy, buffer, bufferOffset);
    // Serialize message field [vz]
    bufferOffset = _serializer.float32(obj.vz, buffer, bufferOffset);
    // Serialize message field [ax]
    bufferOffset = _serializer.float32(obj.ax, buffer, bufferOffset);
    // Serialize message field [ay]
    bufferOffset = _serializer.float32(obj.ay, buffer, bufferOffset);
    // Serialize message field [az]
    bufferOffset = _serializer.float32(obj.az, buffer, bufferOffset);
    // Serialize message field [motor1]
    bufferOffset = _serializer.uint8(obj.motor1, buffer, bufferOffset);
    // Serialize message field [motor2]
    bufferOffset = _serializer.uint8(obj.motor2, buffer, bufferOffset);
    // Serialize message field [motor3]
    bufferOffset = _serializer.uint8(obj.motor3, buffer, bufferOffset);
    // Serialize message field [motor4]
    bufferOffset = _serializer.uint8(obj.motor4, buffer, bufferOffset);
    // Serialize message field [tags_count]
    bufferOffset = _serializer.uint32(obj.tags_count, buffer, bufferOffset);
    // Serialize message field [tags_type]
    bufferOffset = _arraySerializer.uint32(obj.tags_type, buffer, bufferOffset, null);
    // Serialize message field [tags_xc]
    bufferOffset = _arraySerializer.uint32(obj.tags_xc, buffer, bufferOffset, null);
    // Serialize message field [tags_yc]
    bufferOffset = _arraySerializer.uint32(obj.tags_yc, buffer, bufferOffset, null);
    // Serialize message field [tags_width]
    bufferOffset = _arraySerializer.uint32(obj.tags_width, buffer, bufferOffset, null);
    // Serialize message field [tags_height]
    bufferOffset = _arraySerializer.uint32(obj.tags_height, buffer, bufferOffset, null);
    // Serialize message field [tags_orientation]
    bufferOffset = _arraySerializer.float32(obj.tags_orientation, buffer, bufferOffset, null);
    // Serialize message field [tags_distance]
    bufferOffset = _arraySerializer.float32(obj.tags_distance, buffer, bufferOffset, null);
    // Serialize message field [tm]
    bufferOffset = _serializer.float32(obj.tm, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Navdata
    let len;
    let data = new Navdata(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [batteryPercent]
    data.batteryPercent = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [state]
    data.state = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [magX]
    data.magX = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [magY]
    data.magY = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [magZ]
    data.magZ = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [pressure]
    data.pressure = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [temp]
    data.temp = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [wind_speed]
    data.wind_speed = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [wind_angle]
    data.wind_angle = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [wind_comp_angle]
    data.wind_comp_angle = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [rotX]
    data.rotX = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [rotY]
    data.rotY = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [rotZ]
    data.rotZ = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [altd]
    data.altd = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [vx]
    data.vx = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [vy]
    data.vy = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [vz]
    data.vz = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [ax]
    data.ax = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [ay]
    data.ay = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [az]
    data.az = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [motor1]
    data.motor1 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [motor2]
    data.motor2 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [motor3]
    data.motor3 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [motor4]
    data.motor4 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [tags_count]
    data.tags_count = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [tags_type]
    data.tags_type = _arrayDeserializer.uint32(buffer, bufferOffset, null)
    // Deserialize message field [tags_xc]
    data.tags_xc = _arrayDeserializer.uint32(buffer, bufferOffset, null)
    // Deserialize message field [tags_yc]
    data.tags_yc = _arrayDeserializer.uint32(buffer, bufferOffset, null)
    // Deserialize message field [tags_width]
    data.tags_width = _arrayDeserializer.uint32(buffer, bufferOffset, null)
    // Deserialize message field [tags_height]
    data.tags_height = _arrayDeserializer.uint32(buffer, bufferOffset, null)
    // Deserialize message field [tags_orientation]
    data.tags_orientation = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [tags_distance]
    data.tags_distance = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [tm]
    data.tm = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 4 * object.tags_type.length;
    length += 4 * object.tags_xc.length;
    length += 4 * object.tags_yc.length;
    length += 4 * object.tags_width.length;
    length += 4 * object.tags_height.length;
    length += 4 * object.tags_orientation.length;
    length += 4 * object.tags_distance.length;
    return length + 120;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ardrone_autonomy/Navdata';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e1169f766234363125ac62c9a3f87eeb';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    
    # Navdata including the ARDrone 2 specifica sensors
    # (magnetometer, barometer)
    
    # 0 means no battery, 100 means full battery
    float32 batteryPercent
    
    # 0: Unknown, 1: Init, 2: Landed, 3: Flying, 4: Hovering, 5: Test
    # 6: Taking off, 7: Goto Fix Point, 8: Landing, 9: Looping
    # Note: 3,7 seems to discriminate type of flying (isFly = 3 | 7)
    uint32 state
    
    int32 magX
    int32 magY
    int32 magZ
    
    # pressure sensor
    int32 pressure
    
    # apparently, there was a temperature sensor added as well.
    int32 temp
    
    # wind sensing...
    float32 wind_speed
    float32 wind_angle
    float32 wind_comp_angle
    
    # left/right tilt in degrees (rotation about the X axis)
    float32 rotX
    
    # forward/backward tilt in degrees (rotation about the Y axis)
    float32 rotY
    
    # orientation in degrees (rotation about the Z axis)
    float32 rotZ
    
    # estimated altitude (cm)
    int32 altd
    
    # linear velocity (mm/sec)
    float32 vx
    
    # linear velocity (mm/sec)
    float32 vy
    
    # linear velocity (mm/sec)
    float32 vz
    
    #linear accelerations (unit: g)
    float32 ax
    float32 ay
    float32 az
    
    #motor commands (unit 0 to 255)
    uint8 motor1
    uint8 motor2
    uint8 motor3
    uint8 motor4
    
    #Tags in Vision Detectoion
    uint32 tags_count
    uint32[] tags_type
    uint32[] tags_xc
    uint32[] tags_yc
    uint32[] tags_width
    uint32[] tags_height
    float32[] tags_orientation
    float32[] tags_distance
    
    #time stamp
    float32 tm
    
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
    const resolved = new Navdata(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.batteryPercent !== undefined) {
      resolved.batteryPercent = msg.batteryPercent;
    }
    else {
      resolved.batteryPercent = 0.0
    }

    if (msg.state !== undefined) {
      resolved.state = msg.state;
    }
    else {
      resolved.state = 0
    }

    if (msg.magX !== undefined) {
      resolved.magX = msg.magX;
    }
    else {
      resolved.magX = 0
    }

    if (msg.magY !== undefined) {
      resolved.magY = msg.magY;
    }
    else {
      resolved.magY = 0
    }

    if (msg.magZ !== undefined) {
      resolved.magZ = msg.magZ;
    }
    else {
      resolved.magZ = 0
    }

    if (msg.pressure !== undefined) {
      resolved.pressure = msg.pressure;
    }
    else {
      resolved.pressure = 0
    }

    if (msg.temp !== undefined) {
      resolved.temp = msg.temp;
    }
    else {
      resolved.temp = 0
    }

    if (msg.wind_speed !== undefined) {
      resolved.wind_speed = msg.wind_speed;
    }
    else {
      resolved.wind_speed = 0.0
    }

    if (msg.wind_angle !== undefined) {
      resolved.wind_angle = msg.wind_angle;
    }
    else {
      resolved.wind_angle = 0.0
    }

    if (msg.wind_comp_angle !== undefined) {
      resolved.wind_comp_angle = msg.wind_comp_angle;
    }
    else {
      resolved.wind_comp_angle = 0.0
    }

    if (msg.rotX !== undefined) {
      resolved.rotX = msg.rotX;
    }
    else {
      resolved.rotX = 0.0
    }

    if (msg.rotY !== undefined) {
      resolved.rotY = msg.rotY;
    }
    else {
      resolved.rotY = 0.0
    }

    if (msg.rotZ !== undefined) {
      resolved.rotZ = msg.rotZ;
    }
    else {
      resolved.rotZ = 0.0
    }

    if (msg.altd !== undefined) {
      resolved.altd = msg.altd;
    }
    else {
      resolved.altd = 0
    }

    if (msg.vx !== undefined) {
      resolved.vx = msg.vx;
    }
    else {
      resolved.vx = 0.0
    }

    if (msg.vy !== undefined) {
      resolved.vy = msg.vy;
    }
    else {
      resolved.vy = 0.0
    }

    if (msg.vz !== undefined) {
      resolved.vz = msg.vz;
    }
    else {
      resolved.vz = 0.0
    }

    if (msg.ax !== undefined) {
      resolved.ax = msg.ax;
    }
    else {
      resolved.ax = 0.0
    }

    if (msg.ay !== undefined) {
      resolved.ay = msg.ay;
    }
    else {
      resolved.ay = 0.0
    }

    if (msg.az !== undefined) {
      resolved.az = msg.az;
    }
    else {
      resolved.az = 0.0
    }

    if (msg.motor1 !== undefined) {
      resolved.motor1 = msg.motor1;
    }
    else {
      resolved.motor1 = 0
    }

    if (msg.motor2 !== undefined) {
      resolved.motor2 = msg.motor2;
    }
    else {
      resolved.motor2 = 0
    }

    if (msg.motor3 !== undefined) {
      resolved.motor3 = msg.motor3;
    }
    else {
      resolved.motor3 = 0
    }

    if (msg.motor4 !== undefined) {
      resolved.motor4 = msg.motor4;
    }
    else {
      resolved.motor4 = 0
    }

    if (msg.tags_count !== undefined) {
      resolved.tags_count = msg.tags_count;
    }
    else {
      resolved.tags_count = 0
    }

    if (msg.tags_type !== undefined) {
      resolved.tags_type = msg.tags_type;
    }
    else {
      resolved.tags_type = []
    }

    if (msg.tags_xc !== undefined) {
      resolved.tags_xc = msg.tags_xc;
    }
    else {
      resolved.tags_xc = []
    }

    if (msg.tags_yc !== undefined) {
      resolved.tags_yc = msg.tags_yc;
    }
    else {
      resolved.tags_yc = []
    }

    if (msg.tags_width !== undefined) {
      resolved.tags_width = msg.tags_width;
    }
    else {
      resolved.tags_width = []
    }

    if (msg.tags_height !== undefined) {
      resolved.tags_height = msg.tags_height;
    }
    else {
      resolved.tags_height = []
    }

    if (msg.tags_orientation !== undefined) {
      resolved.tags_orientation = msg.tags_orientation;
    }
    else {
      resolved.tags_orientation = []
    }

    if (msg.tags_distance !== undefined) {
      resolved.tags_distance = msg.tags_distance;
    }
    else {
      resolved.tags_distance = []
    }

    if (msg.tm !== undefined) {
      resolved.tm = msg.tm;
    }
    else {
      resolved.tm = 0.0
    }

    return resolved;
    }
};

module.exports = Navdata;
