;; Auto-generated. Do not edit!


(when (boundp 'arm_trajectory::PlanTrajectory)
  (if (not (find-package "ARM_TRAJECTORY"))
    (make-package "ARM_TRAJECTORY"))
  (shadow 'PlanTrajectory (find-package "ARM_TRAJECTORY")))
(unless (find-package "ARM_TRAJECTORY::PLANTRAJECTORY")
  (make-package "ARM_TRAJECTORY::PLANTRAJECTORY"))
(unless (find-package "ARM_TRAJECTORY::PLANTRAJECTORYREQUEST")
  (make-package "ARM_TRAJECTORY::PLANTRAJECTORYREQUEST"))
(unless (find-package "ARM_TRAJECTORY::PLANTRAJECTORYRESPONSE")
  (make-package "ARM_TRAJECTORY::PLANTRAJECTORYRESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))




(defclass arm_trajectory::PlanTrajectoryRequest
  :super ros::object
  :slots (_arm_id _start_pose _target_pose _joint_constraints_min _joint_constraints_max _execution_time _avoid_obstacles _obstacle_frame_ids ))

(defmethod arm_trajectory::PlanTrajectoryRequest
  (:init
   (&key
    ((:arm_id __arm_id) "")
    ((:start_pose __start_pose) (instance geometry_msgs::Pose :init))
    ((:target_pose __target_pose) (instance geometry_msgs::Pose :init))
    ((:joint_constraints_min __joint_constraints_min) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:joint_constraints_max __joint_constraints_max) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:execution_time __execution_time) 0.0)
    ((:avoid_obstacles __avoid_obstacles) nil)
    ((:obstacle_frame_ids __obstacle_frame_ids) (let (r) (dotimes (i 0) (push "" r)) r))
    )
   (send-super :init)
   (setq _arm_id (string __arm_id))
   (setq _start_pose __start_pose)
   (setq _target_pose __target_pose)
   (setq _joint_constraints_min __joint_constraints_min)
   (setq _joint_constraints_max __joint_constraints_max)
   (setq _execution_time (float __execution_time))
   (setq _avoid_obstacles __avoid_obstacles)
   (setq _obstacle_frame_ids __obstacle_frame_ids)
   self)
  (:arm_id
   (&optional __arm_id)
   (if __arm_id (setq _arm_id __arm_id)) _arm_id)
  (:start_pose
   (&rest __start_pose)
   (if (keywordp (car __start_pose))
       (send* _start_pose __start_pose)
     (progn
       (if __start_pose (setq _start_pose (car __start_pose)))
       _start_pose)))
  (:target_pose
   (&rest __target_pose)
   (if (keywordp (car __target_pose))
       (send* _target_pose __target_pose)
     (progn
       (if __target_pose (setq _target_pose (car __target_pose)))
       _target_pose)))
  (:joint_constraints_min
   (&optional __joint_constraints_min)
   (if __joint_constraints_min (setq _joint_constraints_min __joint_constraints_min)) _joint_constraints_min)
  (:joint_constraints_max
   (&optional __joint_constraints_max)
   (if __joint_constraints_max (setq _joint_constraints_max __joint_constraints_max)) _joint_constraints_max)
  (:execution_time
   (&optional __execution_time)
   (if __execution_time (setq _execution_time __execution_time)) _execution_time)
  (:avoid_obstacles
   (&optional (__avoid_obstacles :null))
   (if (not (eq __avoid_obstacles :null)) (setq _avoid_obstacles __avoid_obstacles)) _avoid_obstacles)
  (:obstacle_frame_ids
   (&optional __obstacle_frame_ids)
   (if __obstacle_frame_ids (setq _obstacle_frame_ids __obstacle_frame_ids)) _obstacle_frame_ids)
  (:serialization-length
   ()
   (+
    ;; string _arm_id
    4 (length _arm_id)
    ;; geometry_msgs/Pose _start_pose
    (send _start_pose :serialization-length)
    ;; geometry_msgs/Pose _target_pose
    (send _target_pose :serialization-length)
    ;; float64[] _joint_constraints_min
    (* 8    (length _joint_constraints_min)) 4
    ;; float64[] _joint_constraints_max
    (* 8    (length _joint_constraints_max)) 4
    ;; float64 _execution_time
    8
    ;; bool _avoid_obstacles
    1
    ;; string[] _obstacle_frame_ids
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _obstacle_frame_ids)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _arm_id
       (write-long (length _arm_id) s) (princ _arm_id s)
     ;; geometry_msgs/Pose _start_pose
       (send _start_pose :serialize s)
     ;; geometry_msgs/Pose _target_pose
       (send _target_pose :serialize s)
     ;; float64[] _joint_constraints_min
     (write-long (length _joint_constraints_min) s)
     (dotimes (i (length _joint_constraints_min))
       (sys::poke (elt _joint_constraints_min i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _joint_constraints_max
     (write-long (length _joint_constraints_max) s)
     (dotimes (i (length _joint_constraints_max))
       (sys::poke (elt _joint_constraints_max i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64 _execution_time
       (sys::poke _execution_time (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bool _avoid_obstacles
       (if _avoid_obstacles (write-byte -1 s) (write-byte 0 s))
     ;; string[] _obstacle_frame_ids
     (write-long (length _obstacle_frame_ids) s)
     (dolist (elem _obstacle_frame_ids)
       (write-long (length elem) s) (princ elem s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _arm_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _arm_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; geometry_msgs/Pose _start_pose
     (send _start_pose :deserialize buf ptr-) (incf ptr- (send _start_pose :serialization-length))
   ;; geometry_msgs/Pose _target_pose
     (send _target_pose :deserialize buf ptr-) (incf ptr- (send _target_pose :serialization-length))
   ;; float64[] _joint_constraints_min
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _joint_constraints_min (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _joint_constraints_min i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _joint_constraints_max
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _joint_constraints_max (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _joint_constraints_max i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64 _execution_time
     (setq _execution_time (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool _avoid_obstacles
     (setq _avoid_obstacles (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string[] _obstacle_frame_ids
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _obstacle_frame_ids (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _obstacle_frame_ids i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;;
   self)
  )

(defclass arm_trajectory::PlanTrajectoryResponse
  :super ros::object
  :slots (_success _message _trajectory ))

(defmethod arm_trajectory::PlanTrajectoryResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:message __message) "")
    ((:trajectory __trajectory) (instance arm_trajectory::TrajectoryPath :init))
    )
   (send-super :init)
   (setq _success __success)
   (setq _message (string __message))
   (setq _trajectory __trajectory)
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:trajectory
   (&rest __trajectory)
   (if (keywordp (car __trajectory))
       (send* _trajectory __trajectory)
     (progn
       (if __trajectory (setq _trajectory (car __trajectory)))
       _trajectory)))
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _message
    4 (length _message)
    ;; arm_trajectory/TrajectoryPath _trajectory
    (send _trajectory :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;; arm_trajectory/TrajectoryPath _trajectory
       (send _trajectory :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; arm_trajectory/TrajectoryPath _trajectory
     (send _trajectory :deserialize buf ptr-) (incf ptr- (send _trajectory :serialization-length))
   ;;
   self)
  )

(defclass arm_trajectory::PlanTrajectory
  :super ros::object
  :slots ())

(setf (get arm_trajectory::PlanTrajectory :md5sum-) "495823241b2408e81d569ec04cc0a2d0")
(setf (get arm_trajectory::PlanTrajectory :datatype-) "arm_trajectory/PlanTrajectory")
(setf (get arm_trajectory::PlanTrajectory :request) arm_trajectory::PlanTrajectoryRequest)
(setf (get arm_trajectory::PlanTrajectory :response) arm_trajectory::PlanTrajectoryResponse)

(defmethod arm_trajectory::PlanTrajectoryRequest
  (:response () (instance arm_trajectory::PlanTrajectoryResponse :init)))

(setf (get arm_trajectory::PlanTrajectoryRequest :md5sum-) "495823241b2408e81d569ec04cc0a2d0")
(setf (get arm_trajectory::PlanTrajectoryRequest :datatype-) "arm_trajectory/PlanTrajectoryRequest")
(setf (get arm_trajectory::PlanTrajectoryRequest :definition-)
      "string arm_id                         # ID of the robot arm
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
---
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
")

(setf (get arm_trajectory::PlanTrajectoryResponse :md5sum-) "495823241b2408e81d569ec04cc0a2d0")
(setf (get arm_trajectory::PlanTrajectoryResponse :datatype-) "arm_trajectory/PlanTrajectoryResponse")
(setf (get arm_trajectory::PlanTrajectoryResponse :definition-)
      "string arm_id                         # ID of the robot arm
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
---
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
")



(provide :arm_trajectory/PlanTrajectory "495823241b2408e81d569ec04cc0a2d0")


