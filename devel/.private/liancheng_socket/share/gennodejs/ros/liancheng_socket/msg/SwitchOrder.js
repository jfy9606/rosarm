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

class SwitchOrder {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.station_num = null;
      this.switch_num = null;
      this.case_num = null;
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
        this.station_num = 0;
      }
      if (initObj.hasOwnProperty('switch_num')) {
        this.switch_num = initObj.switch_num
      }
      else {
        this.switch_num = 0;
      }
      if (initObj.hasOwnProperty('case_num')) {
        this.case_num = initObj.case_num
      }
      else {
        this.case_num = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SwitchOrder
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [station_num]
    bufferOffset = _serializer.uint8(obj.station_num, buffer, bufferOffset);
    // Serialize message field [switch_num]
    bufferOffset = _serializer.uint16(obj.switch_num, buffer, bufferOffset);
    // Serialize message field [case_num]
    bufferOffset = _serializer.uint8(obj.case_num, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SwitchOrder
    let len;
    let data = new SwitchOrder(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [station_num]
    data.station_num = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [switch_num]
    data.switch_num = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [case_num]
    data.case_num = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'liancheng_socket/SwitchOrder';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6c01d75e528ea11677c0e88f7ad7f7a1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    
    uint8  station_num
    uint16  switch_num
    uint8  case_num
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
    const resolved = new SwitchOrder(null);
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
      resolved.station_num = 0
    }

    if (msg.switch_num !== undefined) {
      resolved.switch_num = msg.switch_num;
    }
    else {
      resolved.switch_num = 0
    }

    if (msg.case_num !== undefined) {
      resolved.case_num = msg.case_num;
    }
    else {
      resolved.case_num = 0
    }

    return resolved;
    }
};

module.exports = SwitchOrder;
