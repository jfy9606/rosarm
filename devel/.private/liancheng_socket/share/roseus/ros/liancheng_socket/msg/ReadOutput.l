;; Auto-generated. Do not edit!


(when (boundp 'liancheng_socket::ReadOutput)
  (if (not (find-package "LIANCHENG_SOCKET"))
    (make-package "LIANCHENG_SOCKET"))
  (shadow 'ReadOutput (find-package "LIANCHENG_SOCKET")))
(unless (find-package "LIANCHENG_SOCKET::READOUTPUT")
  (make-package "LIANCHENG_SOCKET::READOUTPUT"))

(in-package "ROS")
;;//! \htmlinclude ReadOutput.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass liancheng_socket::ReadOutput
  :super ros::object
  :slots (_header _data_form _data ))

(defmethod liancheng_socket::ReadOutput
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:data_form __data_form) 0)
    ((:data __data) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _data_form (round __data_form))
   (setq _data (round __data))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:data_form
   (&optional __data_form)
   (if __data_form (setq _data_form __data_form)) _data_form)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _data_form
    1
    ;; uint16 _data
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _data_form
       (write-byte _data_form s)
     ;; uint16 _data
       (write-word _data s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _data_form
     (setq _data_form (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint16 _data
     (setq _data (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get liancheng_socket::ReadOutput :md5sum-) "a62420f87b5442275e7e63314bd0b030")
(setf (get liancheng_socket::ReadOutput :datatype-) "liancheng_socket/ReadOutput")
(setf (get liancheng_socket::ReadOutput :definition-)
      "Header header

uint8 data_form
uint16  data
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



(provide :liancheng_socket/ReadOutput "a62420f87b5442275e7e63314bd0b030")


