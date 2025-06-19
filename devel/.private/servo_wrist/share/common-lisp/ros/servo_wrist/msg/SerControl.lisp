; Auto-generated. Do not edit!


(cl:in-package servo_wrist-msg)


;//! \htmlinclude SerControl.msg.html

(cl:defclass <SerControl> (roslisp-msg-protocol:ros-message)
  ((servo_id
    :reader servo_id
    :initarg :servo_id
    :type cl:integer
    :initform 0)
   (target_position
    :reader target_position
    :initarg :target_position
    :type cl:integer
    :initform 0)
   (velocity
    :reader velocity
    :initarg :velocity
    :type cl:integer
    :initform 0)
   (acceleration
    :reader acceleration
    :initarg :acceleration
    :type cl:integer
    :initform 0))
)

(cl:defclass SerControl (<SerControl>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SerControl>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SerControl)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name servo_wrist-msg:<SerControl> is deprecated: use servo_wrist-msg:SerControl instead.")))

(cl:ensure-generic-function 'servo_id-val :lambda-list '(m))
(cl:defmethod servo_id-val ((m <SerControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader servo_wrist-msg:servo_id-val is deprecated.  Use servo_wrist-msg:servo_id instead.")
  (servo_id m))

(cl:ensure-generic-function 'target_position-val :lambda-list '(m))
(cl:defmethod target_position-val ((m <SerControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader servo_wrist-msg:target_position-val is deprecated.  Use servo_wrist-msg:target_position instead.")
  (target_position m))

(cl:ensure-generic-function 'velocity-val :lambda-list '(m))
(cl:defmethod velocity-val ((m <SerControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader servo_wrist-msg:velocity-val is deprecated.  Use servo_wrist-msg:velocity instead.")
  (velocity m))

(cl:ensure-generic-function 'acceleration-val :lambda-list '(m))
(cl:defmethod acceleration-val ((m <SerControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader servo_wrist-msg:acceleration-val is deprecated.  Use servo_wrist-msg:acceleration instead.")
  (acceleration m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SerControl>) ostream)
  "Serializes a message object of type '<SerControl>"
  (cl:let* ((signed (cl:slot-value msg 'servo_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'target_position)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'velocity)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'acceleration)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SerControl>) istream)
  "Deserializes a message object of type '<SerControl>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'servo_id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'target_position) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'velocity) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'acceleration) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SerControl>)))
  "Returns string type for a message object of type '<SerControl>"
  "servo_wrist/SerControl")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SerControl)))
  "Returns string type for a message object of type 'SerControl"
  "servo_wrist/SerControl")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SerControl>)))
  "Returns md5sum for a message object of type '<SerControl>"
  "3a9602bc216147f85b9a19097afea1cd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SerControl)))
  "Returns md5sum for a message object of type 'SerControl"
  "3a9602bc216147f85b9a19097afea1cd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SerControl>)))
  "Returns full string definition for message of type '<SerControl>"
  (cl:format cl:nil "int32 servo_id~%int32 target_position~%int32 velocity~%int32 acceleration~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SerControl)))
  "Returns full string definition for message of type 'SerControl"
  (cl:format cl:nil "int32 servo_id~%int32 target_position~%int32 velocity~%int32 acceleration~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SerControl>))
  (cl:+ 0
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SerControl>))
  "Converts a ROS message object to a list"
  (cl:list 'SerControl
    (cl:cons ':servo_id (servo_id msg))
    (cl:cons ':target_position (target_position msg))
    (cl:cons ':velocity (velocity msg))
    (cl:cons ':acceleration (acceleration msg))
))
