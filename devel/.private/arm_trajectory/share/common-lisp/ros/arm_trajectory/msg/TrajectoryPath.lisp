; Auto-generated. Do not edit!


(cl:in-package arm_trajectory-msg)


;//! \htmlinclude TrajectoryPath.msg.html

(cl:defclass <TrajectoryPath> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (arm_id
    :reader arm_id
    :initarg :arm_id
    :type cl:string
    :initform "")
   (execution_time
    :reader execution_time
    :initarg :execution_time
    :type cl:float
    :initform 0.0)
   (points
    :reader points
    :initarg :points
    :type (cl:vector arm_trajectory-msg:TrajectoryPoint)
   :initform (cl:make-array 0 :element-type 'arm_trajectory-msg:TrajectoryPoint :initial-element (cl:make-instance 'arm_trajectory-msg:TrajectoryPoint))))
)

(cl:defclass TrajectoryPath (<TrajectoryPath>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TrajectoryPath>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TrajectoryPath)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name arm_trajectory-msg:<TrajectoryPath> is deprecated: use arm_trajectory-msg:TrajectoryPath instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <TrajectoryPath>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arm_trajectory-msg:header-val is deprecated.  Use arm_trajectory-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'arm_id-val :lambda-list '(m))
(cl:defmethod arm_id-val ((m <TrajectoryPath>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arm_trajectory-msg:arm_id-val is deprecated.  Use arm_trajectory-msg:arm_id instead.")
  (arm_id m))

(cl:ensure-generic-function 'execution_time-val :lambda-list '(m))
(cl:defmethod execution_time-val ((m <TrajectoryPath>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arm_trajectory-msg:execution_time-val is deprecated.  Use arm_trajectory-msg:execution_time instead.")
  (execution_time m))

(cl:ensure-generic-function 'points-val :lambda-list '(m))
(cl:defmethod points-val ((m <TrajectoryPath>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arm_trajectory-msg:points-val is deprecated.  Use arm_trajectory-msg:points instead.")
  (points m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TrajectoryPath>) ostream)
  "Serializes a message object of type '<TrajectoryPath>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'arm_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'arm_id))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'execution_time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'points))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'points))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TrajectoryPath>) istream)
  "Deserializes a message object of type '<TrajectoryPath>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'arm_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'arm_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'execution_time) (roslisp-utils:decode-double-float-bits bits)))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'points) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'points)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'arm_trajectory-msg:TrajectoryPoint))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TrajectoryPath>)))
  "Returns string type for a message object of type '<TrajectoryPath>"
  "arm_trajectory/TrajectoryPath")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrajectoryPath)))
  "Returns string type for a message object of type 'TrajectoryPath"
  "arm_trajectory/TrajectoryPath")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TrajectoryPath>)))
  "Returns md5sum for a message object of type '<TrajectoryPath>"
  "30a91010797ed089dcc2454df56dea1b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TrajectoryPath)))
  "Returns md5sum for a message object of type 'TrajectoryPath"
  "30a91010797ed089dcc2454df56dea1b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TrajectoryPath>)))
  "Returns full string definition for message of type '<TrajectoryPath>"
  (cl:format cl:nil "Header header~%string arm_id             # ID of the robot arm this trajectory is for~%float64 execution_time    # Total time to execute the trajectory in seconds~%TrajectoryPoint[] points  # Sequence of trajectory points ~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: arm_trajectory/TrajectoryPoint~%Header header~%float64[] joint_positions  # Position for each joint~%float64[] joint_velocities # Velocity for each joint ~%float64[] joint_accelerations # Acceleration for each joint~%geometry_msgs/Pose pose    # End-effector pose corresponding to joint positions ~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TrajectoryPath)))
  "Returns full string definition for message of type 'TrajectoryPath"
  (cl:format cl:nil "Header header~%string arm_id             # ID of the robot arm this trajectory is for~%float64 execution_time    # Total time to execute the trajectory in seconds~%TrajectoryPoint[] points  # Sequence of trajectory points ~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: arm_trajectory/TrajectoryPoint~%Header header~%float64[] joint_positions  # Position for each joint~%float64[] joint_velocities # Velocity for each joint ~%float64[] joint_accelerations # Acceleration for each joint~%geometry_msgs/Pose pose    # End-effector pose corresponding to joint positions ~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TrajectoryPath>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'arm_id))
     8
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'points) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TrajectoryPath>))
  "Converts a ROS message object to a list"
  (cl:list 'TrajectoryPath
    (cl:cons ':header (header msg))
    (cl:cons ':arm_id (arm_id msg))
    (cl:cons ':execution_time (execution_time msg))
    (cl:cons ':points (points msg))
))
