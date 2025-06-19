
(cl:in-package :asdf)

(defsystem "liancheng_socket-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "MotorOrder" :depends-on ("_package_MotorOrder"))
    (:file "_package_MotorOrder" :depends-on ("_package"))
    (:file "ReadOutput" :depends-on ("_package_ReadOutput"))
    (:file "_package_ReadOutput" :depends-on ("_package"))
    (:file "SwitchOrder" :depends-on ("_package_SwitchOrder"))
    (:file "_package_SwitchOrder" :depends-on ("_package"))
  ))