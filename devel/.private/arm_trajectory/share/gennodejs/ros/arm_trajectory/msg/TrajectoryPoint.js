// Auto-generated. Do not edit!

// (in-package arm_trajectory.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class TrajectoryPoint {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.joint_positions = null;
      this.joint_velocities = null;
      this.joint_accelerations = null;
      this.pose = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('joint_positions')) {
        this.joint_positions = initObj.joint_positions
      }
      else {
        this.joint_positions = [];
      }
      if (initObj.hasOwnProperty('joint_velocities')) {
        this.joint_velocities = initObj.joint_velocities
      }
      else {
        this.joint_velocities = [];
      }
      if (initObj.hasOwnProperty('joint_accelerations')) {
        this.joint_accelerations = initObj.joint_accelerations
      }
      else {
        this.joint_accelerations = [];
      }
      if (initObj.hasOwnProperty('pose')) {
        this.pose = initObj.pose
      }
      else {
        this.pose = new geometry_msgs.msg.Pose();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TrajectoryPoint
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [joint_positions]
    bufferOffset = _arraySerializer.float64(obj.joint_positions, buffer, bufferOffset, null);
    // Serialize message field [joint_velocities]
    bufferOffset = _arraySerializer.float64(obj.joint_velocities, buffer, bufferOffset, null);
    // Serialize message field [joint_accelerations]
    bufferOffset = _arraySerializer.float64(obj.joint_accelerations, buffer, bufferOffset, null);
    // Serialize message field [pose]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.pose, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TrajectoryPoint
    let len;
    let data = new TrajectoryPoint(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [joint_positions]
    data.joint_positions = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [joint_velocities]
    data.joint_velocities = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [joint_accelerations]
    data.joint_accelerations = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [pose]
    data.pose = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 8 * object.joint_positions.length;
    length += 8 * object.joint_velocities.length;
    length += 8 * object.joint_accelerations.length;
    return length + 68;
  }

  static datatype() {
    // Returns string type for a message object
    return 'arm_trajectory/TrajectoryPoint';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a22d5b38aae491dace06707bc2138aee';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    float64[] joint_positions  # Position for each joint
    float64[] joint_velocities # Velocity for each joint 
    float64[] joint_accelerations # Acceleration for each joint
    geometry_msgs/Pose pose    # End-effector pose corresponding to joint positions 
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
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TrajectoryPoint(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.joint_positions !== undefined) {
      resolved.joint_positions = msg.joint_positions;
    }
    else {
      resolved.joint_positions = []
    }

    if (msg.joint_velocities !== undefined) {
      resolved.joint_velocities = msg.joint_velocities;
    }
    else {
      resolved.joint_velocities = []
    }

    if (msg.joint_accelerations !== undefined) {
      resolved.joint_accelerations = msg.joint_accelerations;
    }
    else {
      resolved.joint_accelerations = []
    }

    if (msg.pose !== undefined) {
      resolved.pose = geometry_msgs.msg.Pose.Resolve(msg.pose)
    }
    else {
      resolved.pose = new geometry_msgs.msg.Pose()
    }

    return resolved;
    }
};

module.exports = TrajectoryPoint;
