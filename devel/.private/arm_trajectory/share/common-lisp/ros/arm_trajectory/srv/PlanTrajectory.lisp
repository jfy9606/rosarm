; Auto-generated. Do not edit!


(cl:in-package arm_trajectory-srv)


;//! \htmlinclude PlanTrajectory-request.msg.html

(cl:defclass <PlanTrajectory-request> (roslisp-msg-protocol:ros-message)
  ((arm_id
    :reader arm_id
    :initarg :arm_id
    :type cl:string
    :initform "")
   (start_pose
    :reader start_pose
    :initarg :start_pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (target_pose
    :reader target_pose
    :initarg :target_pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (joint_constraints_min
    :reader joint_constraints_min
    :initarg :joint_constraints_min
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (joint_constraints_max
    :reader joint_constraints_max
    :initarg :joint_constraints_max
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (execution_time
    :reader execution_time
    :initarg :execution_time
    :type cl:float
    :initform 0.0)
   (avoid_obstacles
    :reader avoid_obstacles
    :initarg :avoid_obstacles
    :type cl:boolean
    :initform cl:nil)
   (obstacle_frame_ids
    :reader obstacle_frame_ids
    :initarg :obstacle_frame_ids
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass PlanTrajectory-request (<PlanTrajectory-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PlanTrajectory-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PlanTrajectory-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name arm_trajectory-srv:<PlanTrajectory-request> is deprecated: use arm_trajectory-srv:PlanTrajectory-request instead.")))

(cl:ensure-generic-function 'arm_id-val :lambda-list '(m))
(cl:defmethod arm_id-val ((m <PlanTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arm_trajectory-srv:arm_id-val is deprecated.  Use arm_trajectory-srv:arm_id instead.")
  (arm_id m))

(cl:ensure-generic-function 'start_pose-val :lambda-list '(m))
(cl:defmethod start_pose-val ((m <PlanTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arm_trajectory-srv:start_pose-val is deprecated.  Use arm_trajectory-srv:start_pose instead.")
  (start_pose m))

(cl:ensure-generic-function 'target_pose-val :lambda-list '(m))
(cl:defmethod target_pose-val ((m <PlanTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arm_trajectory-srv:target_pose-val is deprecated.  Use arm_trajectory-srv:target_pose instead.")
  (target_pose m))

(cl:ensure-generic-function 'joint_constraints_min-val :lambda-list '(m))
(cl:defmethod joint_constraints_min-val ((m <PlanTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arm_trajectory-srv:joint_constraints_min-val is deprecated.  Use arm_trajectory-srv:joint_constraints_min instead.")
  (joint_constraints_min m))

(cl:ensure-generic-function 'joint_constraints_max-val :lambda-list '(m))
(cl:defmethod joint_constraints_max-val ((m <PlanTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arm_trajectory-srv:joint_constraints_max-val is deprecated.  Use arm_trajectory-srv:joint_constraints_max instead.")
  (joint_constraints_max m))

(cl:ensure-generic-function 'execution_time-val :lambda-list '(m))
(cl:defmethod execution_time-val ((m <PlanTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arm_trajectory-srv:execution_time-val is deprecated.  Use arm_trajectory-srv:execution_time instead.")
  (execution_time m))

(cl:ensure-generic-function 'avoid_obstacles-val :lambda-list '(m))
(cl:defmethod avoid_obstacles-val ((m <PlanTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arm_trajectory-srv:avoid_obstacles-val is deprecated.  Use arm_trajectory-srv:avoid_obstacles instead.")
  (avoid_obstacles m))

(cl:ensure-generic-function 'obstacle_frame_ids-val :lambda-list '(m))
(cl:defmethod obstacle_frame_ids-val ((m <PlanTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arm_trajectory-srv:obstacle_frame_ids-val is deprecated.  Use arm_trajectory-srv:obstacle_frame_ids instead.")
  (obstacle_frame_ids m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PlanTrajectory-request>) ostream)
  "Serializes a message object of type '<PlanTrajectory-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'arm_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'arm_id))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'start_pose) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'target_pose) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'joint_constraints_min))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'joint_constraints_min))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'joint_constraints_max))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'joint_constraints_max))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'execution_time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'avoid_obstacles) 1 0)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'obstacle_frame_ids))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'obstacle_frame_ids))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PlanTrajectory-request>) istream)
  "Deserializes a message object of type '<PlanTrajectory-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'arm_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'arm_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'start_pose) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'target_pose) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'joint_constraints_min) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'joint_constraints_min)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'joint_constraints_max) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'joint_constraints_max)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
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
    (cl:setf (cl:slot-value msg 'avoid_obstacles) (cl:not (cl:zerop (cl:read-byte istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'obstacle_frame_ids) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'obstacle_frame_ids)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PlanTrajectory-request>)))
  "Returns string type for a service object of type '<PlanTrajectory-request>"
  "arm_trajectory/PlanTrajectoryRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlanTrajectory-request)))
  "Returns string type for a service object of type 'PlanTrajectory-request"
  "arm_trajectory/PlanTrajectoryRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PlanTrajectory-request>)))
  "Returns md5sum for a message object of type '<PlanTrajectory-request>"
  "495823241b2408e81d569ec04cc0a2d0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PlanTrajectory-request)))
  "Returns md5sum for a message object of type 'PlanTrajectory-request"
  "495823241b2408e81d569ec04cc0a2d0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PlanTrajectory-request>)))
  "Returns full string definition for message of type '<PlanTrajectory-request>"
  (cl:format cl:nil "string arm_id                         # ID of the robot arm~%geometry_msgs/Pose start_pose          # Start pose of end effector~%geometry_msgs/Pose target_pose         # Target pose of end effector~%float64[] joint_constraints_min        # Minimum joint limits~%float64[] joint_constraints_max        # Maximum joint limits~%float64 execution_time                 # Desired execution time in seconds~%bool avoid_obstacles                   # Whether to avoid obstacles in planning~%string[] obstacle_frame_ids            # IDs of obstacle frames to avoid~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PlanTrajectory-request)))
  "Returns full string definition for message of type 'PlanTrajectory-request"
  (cl:format cl:nil "string arm_id                         # ID of the robot arm~%geometry_msgs/Pose start_pose          # Start pose of end effector~%geometry_msgs/Pose target_pose         # Target pose of end effector~%float64[] joint_constraints_min        # Minimum joint limits~%float64[] joint_constraints_max        # Maximum joint limits~%float64 execution_time                 # Desired execution time in seconds~%bool avoid_obstacles                   # Whether to avoid obstacles in planning~%string[] obstacle_frame_ids            # IDs of obstacle frames to avoid~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PlanTrajectory-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'arm_id))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'start_pose))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'target_pose))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'joint_constraints_min) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'joint_constraints_max) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     8
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'obstacle_frame_ids) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PlanTrajectory-request>))
  "Converts a ROS message object to a list"
  (cl:list 'PlanTrajectory-request
    (cl:cons ':arm_id (arm_id msg))
    (cl:cons ':start_pose (start_pose msg))
    (cl:cons ':target_pose (target_pose msg))
    (cl:cons ':joint_constraints_min (joint_constraints_min msg))
    (cl:cons ':joint_constraints_max (joint_constraints_max msg))
    (cl:cons ':execution_time (execution_time msg))
    (cl:cons ':avoid_obstacles (avoid_obstacles msg))
    (cl:cons ':obstacle_frame_ids (obstacle_frame_ids msg))
))
;//! \htmlinclude PlanTrajectory-response.msg.html

(cl:defclass <PlanTrajectory-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (message
    :reader message
    :initarg :message
    :type cl:string
    :initform "")
   (trajectory
    :reader trajectory
    :initarg :trajectory
    :type arm_trajectory-msg:TrajectoryPath
    :initform (cl:make-instance 'arm_trajectory-msg:TrajectoryPath)))
)

(cl:defclass PlanTrajectory-response (<PlanTrajectory-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PlanTrajectory-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PlanTrajectory-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name arm_trajectory-srv:<PlanTrajectory-response> is deprecated: use arm_trajectory-srv:PlanTrajectory-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <PlanTrajectory-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arm_trajectory-srv:success-val is deprecated.  Use arm_trajectory-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <PlanTrajectory-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arm_trajectory-srv:message-val is deprecated.  Use arm_trajectory-srv:message instead.")
  (message m))

(cl:ensure-generic-function 'trajectory-val :lambda-list '(m))
(cl:defmethod trajectory-val ((m <PlanTrajectory-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arm_trajectory-srv:trajectory-val is deprecated.  Use arm_trajectory-srv:trajectory instead.")
  (trajectory m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PlanTrajectory-response>) ostream)
  "Serializes a message object of type '<PlanTrajectory-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'trajectory) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PlanTrajectory-response>) istream)
  "Deserializes a message object of type '<PlanTrajectory-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'trajectory) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PlanTrajectory-response>)))
  "Returns string type for a service object of type '<PlanTrajectory-response>"
  "arm_trajectory/PlanTrajectoryResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlanTrajectory-response)))
  "Returns string type for a service object of type 'PlanTrajectory-response"
  "arm_trajectory/PlanTrajectoryResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PlanTrajectory-response>)))
  "Returns md5sum for a message object of type '<PlanTrajectory-response>"
  "495823241b2408e81d569ec04cc0a2d0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PlanTrajectory-response)))
  "Returns md5sum for a message object of type 'PlanTrajectory-response"
  "495823241b2408e81d569ec04cc0a2d0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PlanTrajectory-response>)))
  "Returns full string definition for message of type '<PlanTrajectory-response>"
  (cl:format cl:nil "bool success                           # Whether the planning was successful~%string message                         # Status message~%TrajectoryPath trajectory              # The planned trajectory ~%~%================================================================================~%MSG: arm_trajectory/TrajectoryPath~%Header header~%string arm_id             # ID of the robot arm this trajectory is for~%float64 execution_time    # Total time to execute the trajectory in seconds~%TrajectoryPoint[] points  # Sequence of trajectory points ~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: arm_trajectory/TrajectoryPoint~%Header header~%float64[] joint_positions  # Position for each joint~%float64[] joint_velocities # Velocity for each joint ~%float64[] joint_accelerations # Acceleration for each joint~%geometry_msgs/Pose pose    # End-effector pose corresponding to joint positions ~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PlanTrajectory-response)))
  "Returns full string definition for message of type 'PlanTrajectory-response"
  (cl:format cl:nil "bool success                           # Whether the planning was successful~%string message                         # Status message~%TrajectoryPath trajectory              # The planned trajectory ~%~%================================================================================~%MSG: arm_trajectory/TrajectoryPath~%Header header~%string arm_id             # ID of the robot arm this trajectory is for~%float64 execution_time    # Total time to execute the trajectory in seconds~%TrajectoryPoint[] points  # Sequence of trajectory points ~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: arm_trajectory/TrajectoryPoint~%Header header~%float64[] joint_positions  # Position for each joint~%float64[] joint_velocities # Velocity for each joint ~%float64[] joint_accelerations # Acceleration for each joint~%geometry_msgs/Pose pose    # End-effector pose corresponding to joint positions ~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PlanTrajectory-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'trajectory))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PlanTrajectory-response>))
  "Converts a ROS message object to a list"
  (cl:list 'PlanTrajectory-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
    (cl:cons ':trajectory (trajectory msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'PlanTrajectory)))
  'PlanTrajectory-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'PlanTrajectory)))
  'PlanTrajectory-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlanTrajectory)))
  "Returns string type for a service object of type '<PlanTrajectory>"
  "arm_trajectory/PlanTrajectory")