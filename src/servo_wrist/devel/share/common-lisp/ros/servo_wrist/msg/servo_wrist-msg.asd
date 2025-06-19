
(cl:in-package :asdf)

(defsystem "servo_wrist-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "SerControl" :depends-on ("_package_SerControl"))
    (:file "_package_SerControl" :depends-on ("_package"))
  ))