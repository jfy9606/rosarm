// Auto-generated. Do not edit!

// (in-package liancheng_socket.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class MotorOrder {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.station_num = null;
      this.form = null;
      this.vel = null;
      this.vel_ac = null;
      this.vel_de = null;
      this.pos_mode = null;
      this.pos = null;
      this.pos_thr = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('station_num')) {
        this.station_num = initObj.station_num
      }
      else {
        this.station_num = [];
      }
      if (initObj.hasOwnProperty('form')) {
        this.form = initObj.form
      }
      else {
        this.form = [];
      }
      if (initObj.hasOwnProperty('vel')) {
        this.vel = initObj.vel
      }
      else {
        this.vel = [];
      }
      if (initObj.hasOwnProperty('vel_ac')) {
        this.vel_ac = initObj.vel_ac
      }
      else {
        this.vel_ac = [];
      }
      if (initObj.hasOwnProperty('vel_de')) {
        this.vel_de = initObj.vel_de
      }
      else {
        this.vel_de = [];
      }
      if (initObj.hasOwnProperty('pos_mode')) {
        this.pos_mode = initObj.pos_mode
      }
      else {
        this.pos_mode = [];
      }
      if (initObj.hasOwnProperty('pos')) {
        this.pos = initObj.pos
      }
      else {
        this.pos = [];
      }
      if (initObj.hasOwnProperty('pos_thr')) {
        this.pos_thr = initObj.pos_thr
      }
      else {
        this.pos_thr = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MotorOrder
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [station_num]
    bufferOffset = _arraySerializer.uint8(obj.station_num, buffer, bufferOffset, null);
    // Serialize message field [form]
    bufferOffset = _arraySerializer.uint8(obj.form, buffer, bufferOffset, null);
    // Serialize message field [vel]
    bufferOffset = _arraySerializer.int16(obj.vel, buffer, bufferOffset, null);
    // Serialize message field [vel_ac]
    bufferOffset = _arraySerializer.uint16(obj.vel_ac, buffer, bufferOffset, null);
    // Serialize message field [vel_de]
    bufferOffset = _arraySerializer.uint16(obj.vel_de, buffer, bufferOffset, null);
    // Serialize message field [pos_mode]
    bufferOffset = _arraySerializer.bool(obj.pos_mode, buffer, bufferOffset, null);
    // Serialize message field [pos]
    bufferOffset = _arraySerializer.int32(obj.pos, buffer, bufferOffset, null);
    // Serialize message field [pos_thr]
    bufferOffset = _arraySerializer.uint16(obj.pos_thr, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MotorOrder
    let len;
    let data = new MotorOrder(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [station_num]
    data.station_num = _arrayDeserializer.uint8(buffer, bufferOffset, null)
    // Deserialize message field [form]
    data.form = _arrayDeserializer.uint8(buffer, bufferOffset, null)
    // Deserialize message field [vel]
    data.vel = _arrayDeserializer.int16(buffer, bufferOffset, null)
    // Deserialize message field [vel_ac]
    data.vel_ac = _arrayDeserializer.uint16(buffer, bufferOffset, null)
    // Deserialize message field [vel_de]
    data.vel_de = _arrayDeserializer.uint16(buffer, bufferOffset, null)
    // Deserialize message field [pos_mode]
    data.pos_mode = _arrayDeserializer.bool(buffer, bufferOffset, null)
    // Deserialize message field [pos]
    data.pos = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [pos_thr]
    data.pos_thr = _arrayDeserializer.uint16(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += object.station_num.length;
    length += object.form.length;
    length += 2 * object.vel.length;
    length += 2 * object.vel_ac.length;
    length += 2 * object.vel_de.length;
    length += object.pos_mode.length;
    length += 4 * object.pos.length;
    length += 2 * object.pos_thr.length;
    return length + 32;
  }

  static datatype() {
    // Returns string type for a message object
    return 'liancheng_socket/MotorOrder';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '37ba2142a148d7827b5cdcab205b8309';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    
    uint8[] station_num
    uint8[] form
    int16[] vel
    uint16[] vel_ac
    uint16[] vel_de
    bool[] pos_mode
    int32[] pos
    uint16[] pos_thr
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
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MotorOrder(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.station_num !== undefined) {
      resolved.station_num = msg.station_num;
    }
    else {
      resolved.station_num = []
    }

    if (msg.form !== undefined) {
      resolved.form = msg.form;
    }
    else {
      resolved.form = []
    }

    if (msg.vel !== undefined) {
      resolved.vel = msg.vel;
    }
    else {
      resolved.vel = []
    }

    if (msg.vel_ac !== undefined) {
      resolved.vel_ac = msg.vel_ac;
    }
    else {
      resolved.vel_ac = []
    }

    if (msg.vel_de !== undefined) {
      resolved.vel_de = msg.vel_de;
    }
    else {
      resolved.vel_de = []
    }

    if (msg.pos_mode !== undefined) {
      resolved.pos_mode = msg.pos_mode;
    }
    else {
      resolved.pos_mode = []
    }

    if (msg.pos !== undefined) {
      resolved.pos = msg.pos;
    }
    else {
      resolved.pos = []
    }

    if (msg.pos_thr !== undefined) {
      resolved.pos_thr = msg.pos_thr;
    }
    else {
      resolved.pos_thr = []
    }

    return resolved;
    }
};

module.exports = MotorOrder;
