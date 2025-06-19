; Auto-generated. Do not edit!


(cl:in-package liancheng_socket-msg)


;//! \htmlinclude ReadOutput.msg.html

(cl:defclass <ReadOutput> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (data_form
    :reader data_form
    :initarg :data_form
    :type cl:fixnum
    :initform 0)
   (data
    :reader data
    :initarg :data
    :type cl:fixnum
    :initform 0))
)

(cl:defclass ReadOutput (<ReadOutput>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ReadOutput>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ReadOutput)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name liancheng_socket-msg:<ReadOutput> is deprecated: use liancheng_socket-msg:ReadOutput instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ReadOutput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader liancheng_socket-msg:header-val is deprecated.  Use liancheng_socket-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'data_form-val :lambda-list '(m))
(cl:defmethod data_form-val ((m <ReadOutput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader liancheng_socket-msg:data_form-val is deprecated.  Use liancheng_socket-msg:data_form instead.")
  (data_form m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <ReadOutput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader liancheng_socket-msg:data-val is deprecated.  Use liancheng_socket-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ReadOutput>) ostream)
  "Serializes a message object of type '<ReadOutput>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'data_form)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'data)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'data)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ReadOutput>) istream)
  "Deserializes a message object of type '<ReadOutput>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'data_form)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'data)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'data)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ReadOutput>)))
  "Returns string type for a message object of type '<ReadOutput>"
  "liancheng_socket/ReadOutput")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReadOutput)))
  "Returns string type for a message object of type 'ReadOutput"
  "liancheng_socket/ReadOutput")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ReadOutput>)))
  "Returns md5sum for a message object of type '<ReadOutput>"
  "a62420f87b5442275e7e63314bd0b030")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ReadOutput)))
  "Returns md5sum for a message object of type 'ReadOutput"
  "a62420f87b5442275e7e63314bd0b030")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ReadOutput>)))
  "Returns full string definition for message of type '<ReadOutput>"
  (cl:format cl:nil "Header header~%~%uint8 data_form~%uint16  data~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ReadOutput)))
  "Returns full string definition for message of type 'ReadOutput"
  (cl:format cl:nil "Header header~%~%uint8 data_form~%uint16  data~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ReadOutput>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ReadOutput>))
  "Converts a ROS message object to a list"
  (cl:list 'ReadOutput
    (cl:cons ':header (header msg))
    (cl:cons ':data_form (data_form msg))
    (cl:cons ':data (data msg))
))
