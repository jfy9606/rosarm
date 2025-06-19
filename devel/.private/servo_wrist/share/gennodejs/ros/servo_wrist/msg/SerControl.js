// Auto-generated. Do not edit!

// (in-package servo_wrist.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class SerControl {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.servo_id = null;
      this.target_position = null;
      this.velocity = null;
      this.acceleration = null;
    }
    else {
      if (initObj.hasOwnProperty('servo_id')) {
        this.servo_id = initObj.servo_id
      }
      else {
        this.servo_id = 0;
      }
      if (initObj.hasOwnProperty('target_position')) {
        this.target_position = initObj.target_position
      }
      else {
        this.target_position = 0;
      }
      if (initObj.hasOwnProperty('velocity')) {
        this.velocity = initObj.velocity
      }
      else {
        this.velocity = 0;
      }
      if (initObj.hasOwnProperty('acceleration')) {
        this.acceleration = initObj.acceleration
      }
      else {
        this.acceleration = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SerControl
    // Serialize message field [servo_id]
    bufferOffset = _serializer.int32(obj.servo_id, buffer, bufferOffset);
    // Serialize message field [target_position]
    bufferOffset = _serializer.int32(obj.target_position, buffer, bufferOffset);
    // Serialize message field [velocity]
    bufferOffset = _serializer.int32(obj.velocity, buffer, bufferOffset);
    // Serialize message field [acceleration]
    bufferOffset = _serializer.int32(obj.acceleration, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SerControl
    let len;
    let data = new SerControl(null);
    // Deserialize message field [servo_id]
    data.servo_id = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [target_position]
    data.target_position = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [velocity]
    data.velocity = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [acceleration]
    data.acceleration = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'servo_wrist/SerControl';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3a9602bc216147f85b9a19097afea1cd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 servo_id
    int32 target_position
    int32 velocity
    int32 acceleration
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SerControl(null);
    if (msg.servo_id !== undefined) {
      resolved.servo_id = msg.servo_id;
    }
    else {
      resolved.servo_id = 0
    }

    if (msg.target_position !== undefined) {
      resolved.target_position = msg.target_position;
    }
    else {
      resolved.target_position = 0
    }

    if (msg.velocity !== undefined) {
      resolved.velocity = msg.velocity;
    }
    else {
      resolved.velocity = 0
    }

    if (msg.acceleration !== undefined) {
      resolved.acceleration = msg.acceleration;
    }
    else {
      resolved.acceleration = 0
    }

    return resolved;
    }
};

module.exports = SerControl;
