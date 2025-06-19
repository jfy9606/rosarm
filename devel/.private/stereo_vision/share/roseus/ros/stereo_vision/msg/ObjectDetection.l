;; Auto-generated. Do not edit!


(when (boundp 'stereo_vision::ObjectDetection)
  (if (not (find-package "STEREO_VISION"))
    (make-package "STEREO_VISION"))
  (shadow 'ObjectDetection (find-package "STEREO_VISION")))
(unless (find-package "STEREO_VISION::OBJECTDETECTION")
  (make-package "STEREO_VISION::OBJECTDETECTION"))

(in-package "ROS")
;;//! \htmlinclude ObjectDetection.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass stereo_vision::ObjectDetection
  :super ros::object
  :slots (_header _class_name _confidence _x_min _y_min _x_max _y_max _x_3d _y_3d _z_3d _distance ))

(defmethod stereo_vision::ObjectDetection
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:class_name __class_name) "")
    ((:confidence __confidence) 0.0)
    ((:x_min __x_min) 0.0)
    ((:y_min __y_min) 0.0)
    ((:x_max __x_max) 0.0)
    ((:y_max __y_max) 0.0)
    ((:x_3d __x_3d) 0.0)
    ((:y_3d __y_3d) 0.0)
    ((:z_3d __z_3d) 0.0)
    ((:distance __distance) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _class_name (string __class_name))
   (setq _confidence (float __confidence))
   (setq _x_min (float __x_min))
   (setq _y_min (float __y_min))
   (setq _x_max (float __x_max))
   (setq _y_max (float __y_max))
   (setq _x_3d (float __x_3d))
   (setq _y_3d (float __y_3d))
   (setq _z_3d (float __z_3d))
   (setq _distance (float __distance))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:class_name
   (&optional __class_name)
   (if __class_name (setq _class_name __class_name)) _class_name)
  (:confidence
   (&optional __confidence)
   (if __confidence (setq _confidence __confidence)) _confidence)
  (:x_min
   (&optional __x_min)
   (if __x_min (setq _x_min __x_min)) _x_min)
  (:y_min
   (&optional __y_min)
   (if __y_min (setq _y_min __y_min)) _y_min)
  (:x_max
   (&optional __x_max)
   (if __x_max (setq _x_max __x_max)) _x_max)
  (:y_max
   (&optional __y_max)
   (if __y_max (setq _y_max __y_max)) _y_max)
  (:x_3d
   (&optional __x_3d)
   (if __x_3d (setq _x_3d __x_3d)) _x_3d)
  (:y_3d
   (&optional __y_3d)
   (if __y_3d (setq _y_3d __y_3d)) _y_3d)
  (:z_3d
   (&optional __z_3d)
   (if __z_3d (setq _z_3d __z_3d)) _z_3d)
  (:distance
   (&optional __distance)
   (if __distance (setq _distance __distance)) _distance)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _class_name
    4 (length _class_name)
    ;; float32 _confidence
    4
    ;; float32 _x_min
    4
    ;; float32 _y_min
    4
    ;; float32 _x_max
    4
    ;; float32 _y_max
    4
    ;; float32 _x_3d
    4
    ;; float32 _y_3d
    4
    ;; float32 _z_3d
    4
    ;; float32 _distance
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _class_name
       (write-long (length _class_name) s) (princ _class_name s)
     ;; float32 _confidence
       (sys::poke _confidence (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _x_min
       (sys::poke _x_min (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y_min
       (sys::poke _y_min (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _x_max
       (sys::poke _x_max (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y_max
       (sys::poke _y_max (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _x_3d
       (sys::poke _x_3d (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y_3d
       (sys::poke _y_3d (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _z_3d
       (sys::poke _z_3d (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _distance
       (sys::poke _distance (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _class_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _class_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _confidence
     (setq _confidence (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _x_min
     (setq _x_min (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y_min
     (setq _y_min (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _x_max
     (setq _x_max (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y_max
     (setq _y_max (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _x_3d
     (setq _x_3d (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y_3d
     (setq _y_3d (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _z_3d
     (setq _z_3d (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _distance
     (setq _distance (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get stereo_vision::ObjectDetection :md5sum-) "406e440572e8e594fa2c1d30451e5901")
(setf (get stereo_vision::ObjectDetection :datatype-) "stereo_vision/ObjectDetection")
(setf (get stereo_vision::ObjectDetection :definition-)
      "Header header
string class_name
float32 confidence
float32 x_min
float32 y_min
float32 x_max
float32 y_max
float32 x_3d       # X position in 3D space (cm)
float32 y_3d       # Y position in 3D space (cm)
float32 z_3d       # Z position in 3D space (cm)
float32 distance   # Distance to object (cm) 
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

")



(provide :stereo_vision/ObjectDetection "406e440572e8e594fa2c1d30451e5901")


