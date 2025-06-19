;; Auto-generated. Do not edit!


(when (boundp 'liancheng_socket::MotorOrder)
  (if (not (find-package "LIANCHENG_SOCKET"))
    (make-package "LIANCHENG_SOCKET"))
  (shadow 'MotorOrder (find-package "LIANCHENG_SOCKET")))
(unless (find-package "LIANCHENG_SOCKET::MOTORORDER")
  (make-package "LIANCHENG_SOCKET::MOTORORDER"))

(in-package "ROS")
;;//! \htmlinclude MotorOrder.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass liancheng_socket::MotorOrder
  :super ros::object
  :slots (_header _station_num _form _vel _vel_ac _vel_de _pos_mode _pos _pos_thr ))

(defmethod liancheng_socket::MotorOrder
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:station_num __station_num) (make-array 0 :initial-element 0 :element-type :char))
    ((:form __form) (make-array 0 :initial-element 0 :element-type :char))
    ((:vel __vel) (make-array 0 :initial-element 0 :element-type :integer))
    ((:vel_ac __vel_ac) (make-array 0 :initial-element 0 :element-type :integer))
    ((:vel_de __vel_de) (make-array 0 :initial-element 0 :element-type :integer))
    ((:pos_mode __pos_mode) (let (r) (dotimes (i 0) (push nil r)) r))
    ((:pos __pos) (make-array 0 :initial-element 0 :element-type :integer))
    ((:pos_thr __pos_thr) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _header __header)
   (setq _station_num __station_num)
   (setq _form __form)
   (setq _vel __vel)
   (setq _vel_ac __vel_ac)
   (setq _vel_de __vel_de)
   (setq _pos_mode __pos_mode)
   (setq _pos __pos)
   (setq _pos_thr __pos_thr)
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
  (:form
   (&optional __form)
   (if __form (setq _form __form)) _form)
  (:vel
   (&optional __vel)
   (if __vel (setq _vel __vel)) _vel)
  (:vel_ac
   (&optional __vel_ac)
   (if __vel_ac (setq _vel_ac __vel_ac)) _vel_ac)
  (:vel_de
   (&optional __vel_de)
   (if __vel_de (setq _vel_de __vel_de)) _vel_de)
  (:pos_mode
   (&optional __pos_mode)
   (if __pos_mode (setq _pos_mode __pos_mode)) _pos_mode)
  (:pos
   (&optional __pos)
   (if __pos (setq _pos __pos)) _pos)
  (:pos_thr
   (&optional __pos_thr)
   (if __pos_thr (setq _pos_thr __pos_thr)) _pos_thr)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8[] _station_num
    (* 1    (length _station_num)) 4
    ;; uint8[] _form
    (* 1    (length _form)) 4
    ;; int16[] _vel
    (* 2    (length _vel)) 4
    ;; uint16[] _vel_ac
    (* 2    (length _vel_ac)) 4
    ;; uint16[] _vel_de
    (* 2    (length _vel_de)) 4
    ;; bool[] _pos_mode
    (* 1    (length _pos_mode)) 4
    ;; int32[] _pos
    (* 4    (length _pos)) 4
    ;; uint16[] _pos_thr
    (* 2    (length _pos_thr)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8[] _station_num
     (write-long (length _station_num) s)
     (princ _station_num s)
     ;; uint8[] _form
     (write-long (length _form) s)
     (princ _form s)
     ;; int16[] _vel
     (write-long (length _vel) s)
     (dotimes (i (length _vel))
       (write-word (elt _vel i) s)
       )
     ;; uint16[] _vel_ac
     (write-long (length _vel_ac) s)
     (dotimes (i (length _vel_ac))
       (write-word (elt _vel_ac i) s)
       )
     ;; uint16[] _vel_de
     (write-long (length _vel_de) s)
     (dotimes (i (length _vel_de))
       (write-word (elt _vel_de i) s)
       )
     ;; bool[] _pos_mode
     (write-long (length _pos_mode) s)
     (dotimes (i (length _pos_mode))
       (if (elt _pos_mode i) (write-byte -1 s) (write-byte 0 s))
       )
     ;; int32[] _pos
     (write-long (length _pos) s)
     (dotimes (i (length _pos))
       (write-long (elt _pos i) s)
       )
     ;; uint16[] _pos_thr
     (write-long (length _pos_thr) s)
     (dotimes (i (length _pos_thr))
       (write-word (elt _pos_thr i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8[] _station_num
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _station_num (make-array n :element-type :char))
     (replace _station_num buf :start2 ptr-) (incf ptr- n))
   ;; uint8[] _form
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _form (make-array n :element-type :char))
     (replace _form buf :start2 ptr-) (incf ptr- n))
   ;; int16[] _vel
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _vel (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _vel i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;; uint16[] _vel_ac
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _vel_ac (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _vel_ac i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;; uint16[] _vel_de
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _vel_de (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _vel_de i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;; bool[] _pos_mode
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _pos_mode (make-list n))
     (dotimes (i n)
     (setf (elt _pos_mode i) (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
     ))
   ;; int32[] _pos
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _pos (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _pos i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; uint16[] _pos_thr
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _pos_thr (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _pos_thr i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;;
   self)
  )

(setf (get liancheng_socket::MotorOrder :md5sum-) "37ba2142a148d7827b5cdcab205b8309")
(setf (get liancheng_socket::MotorOrder :datatype-) "liancheng_socket/MotorOrder")
(setf (get liancheng_socket::MotorOrder :definition-)
      "Header header

uint8[] station_num
uint8[] form
int16[] vel
uint16[] vel_ac
uint16[] vel_de
bool[] pos_mode
int32[] pos
uint16[] pos_thr
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



(provide :liancheng_socket/MotorOrder "37ba2142a148d7827b5cdcab205b8309")


