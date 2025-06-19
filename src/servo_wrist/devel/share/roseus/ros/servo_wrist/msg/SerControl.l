;; Auto-generated. Do not edit!


(when (boundp 'servo_wrist::SerControl)
  (if (not (find-package "SERVO_WRIST"))
    (make-package "SERVO_WRIST"))
  (shadow 'SerControl (find-package "SERVO_WRIST")))
(unless (find-package "SERVO_WRIST::SERCONTROL")
  (make-package "SERVO_WRIST::SERCONTROL"))

(in-package "ROS")
;;//! \htmlinclude SerControl.msg.html


(defclass servo_wrist::SerControl
  :super ros::object
  :slots (_servo_id _target_position _velocity _acceleration ))

(defmethod servo_wrist::SerControl
  (:init
   (&key
    ((:servo_id __servo_id) 0)
    ((:target_position __target_position) 0)
    ((:velocity __velocity) 0)
    ((:acceleration __acceleration) 0)
    )
   (send-super :init)
   (setq _servo_id (round __servo_id))
   (setq _target_position (round __target_position))
   (setq _velocity (round __velocity))
   (setq _acceleration (round __acceleration))
   self)
  (:servo_id
   (&optional __servo_id)
   (if __servo_id (setq _servo_id __servo_id)) _servo_id)
  (:target_position
   (&optional __target_position)
   (if __target_position (setq _target_position __target_position)) _target_position)
  (:velocity
   (&optional __velocity)
   (if __velocity (setq _velocity __velocity)) _velocity)
  (:acceleration
   (&optional __acceleration)
   (if __acceleration (setq _acceleration __acceleration)) _acceleration)
  (:serialization-length
   ()
   (+
    ;; int32 _servo_id
    4
    ;; int32 _target_position
    4
    ;; int32 _velocity
    4
    ;; int32 _acceleration
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _servo_id
       (write-long _servo_id s)
     ;; int32 _target_position
       (write-long _target_position s)
     ;; int32 _velocity
       (write-long _velocity s)
     ;; int32 _acceleration
       (write-long _acceleration s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _servo_id
     (setq _servo_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _target_position
     (setq _target_position (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _velocity
     (setq _velocity (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _acceleration
     (setq _acceleration (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get servo_wrist::SerControl :md5sum-) "3a9602bc216147f85b9a19097afea1cd")
(setf (get servo_wrist::SerControl :datatype-) "servo_wrist/SerControl")
(setf (get servo_wrist::SerControl :definition-)
      "int32 servo_id
int32 target_position
int32 velocity
int32 acceleration


")



(provide :servo_wrist/SerControl "3a9602bc216147f85b9a19097afea1cd")


