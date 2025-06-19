;; Auto-generated. Do not edit!


(when (boundp 'arm_trajectory::TrajectoryPoint)
  (if (not (find-package "ARM_TRAJECTORY"))
    (make-package "ARM_TRAJECTORY"))
  (shadow 'TrajectoryPoint (find-package "ARM_TRAJECTORY")))
(unless (find-package "ARM_TRAJECTORY::TRAJECTORYPOINT")
  (make-package "ARM_TRAJECTORY::TRAJECTORYPOINT"))

(in-package "ROS")
;;//! \htmlinclude TrajectoryPoint.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass arm_trajectory::TrajectoryPoint
  :super ros::object
  :slots (_header _joint_positions _joint_velocities _joint_accelerations _pose ))

(defmethod arm_trajectory::TrajectoryPoint
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:joint_positions __joint_positions) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:joint_velocities __joint_velocities) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:joint_accelerations __joint_accelerations) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:pose __pose) (instance geometry_msgs::Pose :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _joint_positions __joint_positions)
   (setq _joint_velocities __joint_velocities)
   (setq _joint_accelerations __joint_accelerations)
   (setq _pose __pose)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:joint_positions
   (&optional __joint_positions)
   (if __joint_positions (setq _joint_positions __joint_positions)) _joint_positions)
  (:joint_velocities
   (&optional __joint_velocities)
   (if __joint_velocities (setq _joint_velocities __joint_velocities)) _joint_velocities)
  (:joint_accelerations
   (&optional __joint_accelerations)
   (if __joint_accelerations (setq _joint_accelerations __joint_accelerations)) _joint_accelerations)
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64[] _joint_positions
    (* 8    (length _joint_positions)) 4
    ;; float64[] _joint_velocities
    (* 8    (length _joint_velocities)) 4
    ;; float64[] _joint_accelerations
    (* 8    (length _joint_accelerations)) 4
    ;; geometry_msgs/Pose _pose
    (send _pose :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64[] _joint_positions
     (write-long (length _joint_positions) s)
     (dotimes (i (length _joint_positions))
       (sys::poke (elt _joint_positions i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _joint_velocities
     (write-long (length _joint_velocities) s)
     (dotimes (i (length _joint_velocities))
       (sys::poke (elt _joint_velocities i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _joint_accelerations
     (write-long (length _joint_accelerations) s)
     (dotimes (i (length _joint_accelerations))
       (sys::poke (elt _joint_accelerations i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; geometry_msgs/Pose _pose
       (send _pose :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64[] _joint_positions
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _joint_positions (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _joint_positions i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _joint_velocities
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _joint_velocities (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _joint_velocities i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _joint_accelerations
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _joint_accelerations (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _joint_accelerations i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; geometry_msgs/Pose _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;;
   self)
  )

(setf (get arm_trajectory::TrajectoryPoint :md5sum-) "a22d5b38aae491dace06707bc2138aee")
(setf (get arm_trajectory::TrajectoryPoint :datatype-) "arm_trajectory/TrajectoryPoint")
(setf (get arm_trajectory::TrajectoryPoint :definition-)
      "Header header
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

")



(provide :arm_trajectory/TrajectoryPoint "a22d5b38aae491dace06707bc2138aee")


