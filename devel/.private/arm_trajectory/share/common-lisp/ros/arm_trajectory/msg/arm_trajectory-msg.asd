
(cl:in-package :asdf)

(defsystem "arm_trajectory-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "TrajectoryPath" :depends-on ("_package_TrajectoryPath"))
    (:file "_package_TrajectoryPath" :depends-on ("_package"))
    (:file "TrajectoryPoint" :depends-on ("_package_TrajectoryPoint"))
    (:file "_package_TrajectoryPoint" :depends-on ("_package"))
  ))