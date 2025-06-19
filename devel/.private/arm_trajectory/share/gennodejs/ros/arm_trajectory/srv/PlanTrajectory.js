// Auto-generated. Do not edit!

// (in-package arm_trajectory.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

let TrajectoryPath = require('../msg/TrajectoryPath.js');

//-----------------------------------------------------------

class PlanTrajectoryRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.arm_id = null;
      this.start_pose = null;
      this.target_pose = null;
      this.joint_constraints_min = null;
      this.joint_constraints_max = null;
      this.execution_time = null;
      this.avoid_obstacles = null;
      this.obstacle_frame_ids = null;
    }
    else {
      if (initObj.hasOwnProperty('arm_id')) {
        this.arm_id = initObj.arm_id
      }
      else {
        this.arm_id = '';
      }
      if (initObj.hasOwnProperty('start_pose')) {
        this.start_pose = initObj.start_pose
      }
      else {
        this.start_pose = new geometry_msgs.msg.Pose();
      }
      if (initObj.hasOwnProperty('target_pose')) {
        this.target_pose = initObj.target_pose
      }
      else {
        this.target_pose = new geometry_msgs.msg.Pose();
      }
      if (initObj.hasOwnProperty('joint_constraints_min')) {
        this.joint_constraints_min = initObj.joint_constraints_min
      }
      else {
        this.joint_constraints_min = [];
      }
      if (initObj.hasOwnProperty('joint_constraints_max')) {
        this.joint_constraints_max = initObj.joint_constraints_max
      }
      else {
        this.joint_constraints_max = [];
      }
      if (initObj.hasOwnProperty('execution_time')) {
        this.execution_time = initObj.execution_time
      }
      else {
        this.execution_time = 0.0;
      }
      if (initObj.hasOwnProperty('avoid_obstacles')) {
        this.avoid_obstacles = initObj.avoid_obstacles
      }
      else {
        this.avoid_obstacles = false;
      }
      if (initObj.hasOwnProperty('obstacle_frame_ids')) {
        this.obstacle_frame_ids = initObj.obstacle_frame_ids
      }
      else {
        this.obstacle_frame_ids = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PlanTrajectoryRequest
    // Serialize message field [arm_id]
    bufferOffset = _serializer.string(obj.arm_id, buffer, bufferOffset);
    // Serialize message field [start_pose]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.start_pose, buffer, bufferOffset);
    // Serialize message field [target_pose]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.target_pose, buffer, bufferOffset);
    // Serialize message field [joint_constraints_min]
    bufferOffset = _arraySerializer.float64(obj.joint_constraints_min, buffer, bufferOffset, null);
    // Serialize message field [joint_constraints_max]
    bufferOffset = _arraySerializer.float64(obj.joint_constraints_max, buffer, bufferOffset, null);
    // Serialize message field [execution_time]
    bufferOffset = _serializer.float64(obj.execution_time, buffer, bufferOffset);
    // Serialize message field [avoid_obstacles]
    bufferOffset = _serializer.bool(obj.avoid_obstacles, buffer, bufferOffset);
    // Serialize message field [obstacle_frame_ids]
    bufferOffset = _arraySerializer.string(obj.obstacle_frame_ids, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PlanTrajectoryRequest
    let len;
    let data = new PlanTrajectoryRequest(null);
    // Deserialize message field [arm_id]
    data.arm_id = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [start_pose]
    data.start_pose = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    // Deserialize message field [target_pose]
    data.target_pose = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    // Deserialize message field [joint_constraints_min]
    data.joint_constraints_min = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [joint_constraints_max]
    data.joint_constraints_max = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [execution_time]
    data.execution_time = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [avoid_obstacles]
    data.avoid_obstacles = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [obstacle_frame_ids]
    data.obstacle_frame_ids = _arrayDeserializer.string(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.arm_id);
    length += 8 * object.joint_constraints_min.length;
    length += 8 * object.joint_constraints_max.length;
    object.obstacle_frame_ids.forEach((val) => {
      length += 4 + _getByteLength(val);
    });
    return length + 137;
  }

  static datatype() {
    // Returns string type for a service object
    return 'arm_trajectory/PlanTrajectoryRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '32ee205d6ef11a3c9406111a31dd3c5d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string arm_id                         # ID of the robot arm
    geometry_msgs/Pose start_pose          # Start pose of end effector
    geometry_msgs/Pose target_pose         # Target pose of end effector
    float64[] joint_constraints_min        # Minimum joint limits
    float64[] joint_constraints_max        # Maximum joint limits
    float64 execution_time                 # Desired execution time in seconds
    bool avoid_obstacles                   # Whether to avoid obstacles in planning
    string[] obstacle_frame_ids            # IDs of obstacle frames to avoid
    
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
    const resolved = new PlanTrajectoryRequest(null);
    if (msg.arm_id !== undefined) {
      resolved.arm_id = msg.arm_id;
    }
    else {
      resolved.arm_id = ''
    }

    if (msg.start_pose !== undefined) {
      resolved.start_pose = geometry_msgs.msg.Pose.Resolve(msg.start_pose)
    }
    else {
      resolved.start_pose = new geometry_msgs.msg.Pose()
    }

    if (msg.target_pose !== undefined) {
      resolved.target_pose = geometry_msgs.msg.Pose.Resolve(msg.target_pose)
    }
    else {
      resolved.target_pose = new geometry_msgs.msg.Pose()
    }

    if (msg.joint_constraints_min !== undefined) {
      resolved.joint_constraints_min = msg.joint_constraints_min;
    }
    else {
      resolved.joint_constraints_min = []
    }

    if (msg.joint_constraints_max !== undefined) {
      resolved.joint_constraints_max = msg.joint_constraints_max;
    }
    else {
      resolved.joint_constraints_max = []
    }

    if (msg.execution_time !== undefined) {
      resolved.execution_time = msg.execution_time;
    }
    else {
      resolved.execution_time = 0.0
    }

    if (msg.avoid_obstacles !== undefined) {
      resolved.avoid_obstacles = msg.avoid_obstacles;
    }
    else {
      resolved.avoid_obstacles = false
    }

    if (msg.obstacle_frame_ids !== undefined) {
      resolved.obstacle_frame_ids = msg.obstacle_frame_ids;
    }
    else {
      resolved.obstacle_frame_ids = []
    }

    return resolved;
    }
};

class PlanTrajectoryResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
      this.message = null;
      this.trajectory = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
      if (initObj.hasOwnProperty('message')) {
        this.message = initObj.message
      }
      else {
        this.message = '';
      }
      if (initObj.hasOwnProperty('trajectory')) {
        this.trajectory = initObj.trajectory
      }
      else {
        this.trajectory = new TrajectoryPath();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PlanTrajectoryResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    // Serialize message field [trajectory]
    bufferOffset = TrajectoryPath.serialize(obj.trajectory, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PlanTrajectoryResponse
    let len;
    let data = new PlanTrajectoryResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [message]
    data.message = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [trajectory]
    data.trajectory = TrajectoryPath.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.message);
    length += TrajectoryPath.getMessageSize(object.trajectory);
    return length + 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'arm_trajectory/PlanTrajectoryResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '99cffc52e55277bb184fe2948c14645e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success                           # Whether the planning was successful
    string message                         # Status message
    TrajectoryPath trajectory              # The planned trajectory 
    
    ================================================================================
    MSG: arm_trajectory/TrajectoryPath
    Header header
    string arm_id             # ID of the robot arm this trajectory is for
    float64 execution_time    # Total time to execute the trajectory in seconds
    TrajectoryPoint[] points  # Sequence of trajectory points 
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
    MSG: arm_trajectory/TrajectoryPoint
    Header header
    float64[] joint_positions  # Position for each joint
    float64[] joint_velocities # Velocity for each joint 
    float64[] joint_accelerations # Acceleration for each joint
    geometry_msgs/Pose pose    # End-effector pose corresponding to joint positions 
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
    const resolved = new PlanTrajectoryResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    if (msg.message !== undefined) {
      resolved.message = msg.message;
    }
    else {
      resolved.message = ''
    }

    if (msg.trajectory !== undefined) {
      resolved.trajectory = TrajectoryPath.Resolve(msg.trajectory)
    }
    else {
      resolved.trajectory = new TrajectoryPath()
    }

    return resolved;
    }
};

module.exports = {
  Request: PlanTrajectoryRequest,
  Response: PlanTrajectoryResponse,
  md5sum() { return '495823241b2408e81d569ec04cc0a2d0'; },
  datatype() { return 'arm_trajectory/PlanTrajectory'; }
};
