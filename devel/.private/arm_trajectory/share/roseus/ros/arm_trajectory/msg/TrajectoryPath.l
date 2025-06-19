;; Auto-generated. Do not edit!


(when (boundp 'arm_trajectory::TrajectoryPath)
  (if (not (find-package "ARM_TRAJECTORY"))
    (make-package "ARM_TRAJECTORY"))
  (shadow 'TrajectoryPath (find-package "ARM_TRAJECTORY")))
(unless (find-package "ARM_TRAJECTORY::TRAJECTORYPATH")
  (make-package "ARM_TRAJECTORY::TRAJECTORYPATH"))

(in-package "ROS")
;;//! \htmlinclude TrajectoryPath.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass arm_trajectory::TrajectoryPath
  :super ros::object
  :slots (_header _arm_id _execution_time _points ))

(defmethod arm_trajectory::TrajectoryPath
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:arm_id __arm_id) "")
    ((:execution_time __execution_time) 0.0)
    ((:points __points) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _arm_id (string __arm_id))
   (setq _execution_time (float __execution_time))
   (setq _points __points)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:arm_id
   (&optional __arm_id)
   (if __arm_id (setq _arm_id __arm_id)) _arm_id)
  (:execution_time
   (&optional __execution_time)
   (if __execution_time (setq _execution_time __execution_time)) _execution_time)
  (:points
   (&rest __points)
   (if (keywordp (car __points))
       (send* _points __points)
     (progn
       (if __points (setq _points (car __points)))
       _points)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _arm_id
    4 (length _arm_id)
    ;; float64 _execution_time
    8
    ;; arm_trajectory/TrajectoryPoint[] _points
    (apply #'+ (send-all _points :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _arm_id
       (write-long (length _arm_id) s) (princ _arm_id s)
     ;; float64 _execution_time
       (sys::poke _execution_time (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; arm_trajectory/TrajectoryPoint[] _points
     (write-long (length _points) s)
     (dolist (elem _points)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _arm_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _arm_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _execution_time
     (setq _execution_time (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; arm_trajectory/TrajectoryPoint[] _points
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _points (let (r) (dotimes (i n) (push (instance arm_trajectory::TrajectoryPoint :init) r)) r))
     (dolist (elem- _points)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get arm_trajectory::TrajectoryPath :md5sum-) "30a91010797ed089dcc2454df56dea1b")
(setf (get arm_trajectory::TrajectoryPath :datatype-) "arm_trajectory/TrajectoryPath")
(setf (get arm_trajectory::TrajectoryPath :definition-)
      "Header header
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



(provide :arm_trajectory/TrajectoryPath "30a91010797ed089dcc2454df56dea1b")


