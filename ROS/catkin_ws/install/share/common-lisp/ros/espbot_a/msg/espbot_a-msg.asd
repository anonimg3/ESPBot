
(cl:in-package :asdf)

(defsystem "espbot_a-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Servo_Position" :depends-on ("_package_Servo_Position"))
    (:file "_package_Servo_Position" :depends-on ("_package"))
  ))