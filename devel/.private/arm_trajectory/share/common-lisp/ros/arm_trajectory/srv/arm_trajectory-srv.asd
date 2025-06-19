
(cl:in-package :asdf)

(defsystem "arm_trajectory-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :arm_trajectory-msg
               :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "PlanTrajectory" :depends-on ("_package_PlanTrajectory"))
    (:file "_package_PlanTrajectory" :depends-on ("_package"))
  ))