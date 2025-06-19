; Auto-generated. Do not edit!


(cl:in-package liancheng_socket-msg)


;//! \htmlinclude SwitchOrder.msg.html

(cl:defclass <SwitchOrder> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (station_num
    :reader station_num
    :initarg :station_num
    :type cl:fixnum
    :initform 0)
   (switch_num
    :reader switch_num
    :initarg :switch_num
    :type cl:fixnum
    :initform 0)
   (case_num
    :reader case_num
    :initarg :case_num
    :type cl:fixnum
    :initform 0))
)

(cl:defclass SwitchOrder (<SwitchOrder>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SwitchOrder>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SwitchOrder)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name liancheng_socket-msg:<SwitchOrder> is deprecated: use liancheng_socket-msg:SwitchOrder instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <SwitchOrder>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader liancheng_socket-msg:header-val is deprecated.  Use liancheng_socket-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'station_num-val :lambda-list '(m))
(cl:defmethod station_num-val ((m <SwitchOrder>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader liancheng_socket-msg:station_num-val is deprecated.  Use liancheng_socket-msg:station_num instead.")
  (station_num m))

(cl:ensure-generic-function 'switch_num-val :lambda-list '(m))
(cl:defmethod switch_num-val ((m <SwitchOrder>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader liancheng_socket-msg:switch_num-val is deprecated.  Use liancheng_socket-msg:switch_num instead.")
  (switch_num m))

(cl:ensure-generic-function 'case_num-val :lambda-list '(m))
(cl:defmethod case_num-val ((m <SwitchOrder>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader liancheng_socket-msg:case_num-val is deprecated.  Use liancheng_socket-msg:case_num instead.")
  (case_num m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SwitchOrder>) ostream)
  "Serializes a message object of type '<SwitchOrder>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'station_num)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'switch_num)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'switch_num)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'case_num)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SwitchOrder>) istream)
  "Deserializes a message object of type '<SwitchOrder>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'station_num)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'switch_num)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'switch_num)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'case_num)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SwitchOrder>)))
  "Returns string type for a message object of type '<SwitchOrder>"
  "liancheng_socket/SwitchOrder")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SwitchOrder)))
  "Returns string type for a message object of type 'SwitchOrder"
  "liancheng_socket/SwitchOrder")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SwitchOrder>)))
  "Returns md5sum for a message object of type '<SwitchOrder>"
  "6c01d75e528ea11677c0e88f7ad7f7a1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SwitchOrder)))
  "Returns md5sum for a message object of type 'SwitchOrder"
  "6c01d75e528ea11677c0e88f7ad7f7a1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SwitchOrder>)))
  "Returns full string definition for message of type '<SwitchOrder>"
  (cl:format cl:nil "Header header~%~%uint8  station_num~%uint16  switch_num~%uint8  case_num~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SwitchOrder)))
  "Returns full string definition for message of type 'SwitchOrder"
  (cl:format cl:nil "Header header~%~%uint8  station_num~%uint16  switch_num~%uint8  case_num~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SwitchOrder>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     2
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SwitchOrder>))
  "Converts a ROS message object to a list"
  (cl:list 'SwitchOrder
    (cl:cons ':header (header msg))
    (cl:cons ':station_num (station_num msg))
    (cl:cons ':switch_num (switch_num msg))
    (cl:cons ':case_num (case_num msg))
))
