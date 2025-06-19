;; Auto-generated. Do not edit!


(when (boundp 'liancheng_socket::SwitchOrder)
  (if (not (find-package "LIANCHENG_SOCKET"))
    (make-package "LIANCHENG_SOCKET"))
  (shadow 'SwitchOrder (find-package "LIANCHENG_SOCKET")))
(unless (find-package "LIANCHENG_SOCKET::SWITCHORDER")
  (make-package "LIANCHENG_SOCKET::SWITCHORDER"))

(in-package "ROS")
;;//! \htmlinclude SwitchOrder.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass liancheng_socket::SwitchOrder
  :super ros::object
  :slots (_header _station_num _switch_num _case_num ))

(defmethod liancheng_socket::SwitchOrder
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:station_num __station_num) 0)
    ((:switch_num __switch_num) 0)
    ((:case_num __case_num) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _station_num (round __station_num))
   (setq _switch_num (round __switch_num))
   (setq _case_num (round __case_num))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:station_num
   (&optional __station_num)
   (if __station_num (setq _station_num __station_num)) _station_num)
  (:switch_num
   (&optional __switch_num)
   (if __switch_num (setq _switch_num __switch_num)) _switch_num)
  (:case_num
   (&optional __case_num)
   (if __case_num (setq _case_num __case_num)) _case_num)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _station_num
    1
    ;; uint16 _switch_num
    2
    ;; uint8 _case_num
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _station_num
       (write-byte _station_num s)
     ;; uint16 _switch_num
       (write-word _switch_num s)
     ;; uint8 _case_num
       (write-byte _case_num s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _station_num
     (setq _station_num (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint16 _switch_num
     (setq _switch_num (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint8 _case_num
     (setq _case_num (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get liancheng_socket::SwitchOrder :md5sum-) "6c01d75e528ea11677c0e88f7ad7f7a1")
(setf (get liancheng_socket::SwitchOrder :datatype-) "liancheng_socket/SwitchOrder")
(setf (get liancheng_socket::SwitchOrder :definition-)
      "Header header

uint8  station_num
uint16  switch_num
uint8  case_num
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



(provide :liancheng_socket/SwitchOrder "6c01d75e528ea11677c0e88f7ad7f7a1")


