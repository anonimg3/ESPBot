;; Auto-generated. Do not edit!


(when (boundp 'espbot_a::Servo_Position)
  (if (not (find-package "ESPBOT_A"))
    (make-package "ESPBOT_A"))
  (shadow 'Servo_Position (find-package "ESPBOT_A")))
(unless (find-package "ESPBOT_A::SERVO_POSITION")
  (make-package "ESPBOT_A::SERVO_POSITION"))

(in-package "ROS")
;;//! \htmlinclude Servo_Position.msg.html


(defclass espbot_a::Servo_Position
  :super ros::object
  :slots (_angle ))

(defmethod espbot_a::Servo_Position
  (:init
   (&key
    ((:angle __angle) 0.0)
    )
   (send-super :init)
   (setq _angle (float __angle))
   self)
  (:angle
   (&optional __angle)
   (if __angle (setq _angle __angle)) _angle)
  (:serialization-length
   ()
   (+
    ;; float64 _angle
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _angle
       (sys::poke _angle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _angle
     (setq _angle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get espbot_a::Servo_Position :md5sum-) "4edb55038e2b888976a0c0c56935341c")
(setf (get espbot_a::Servo_Position :datatype-) "espbot_a/Servo_Position")
(setf (get espbot_a::Servo_Position :definition-)
      "float64 angle
")



(provide :espbot_a/Servo_Position "4edb55038e2b888976a0c0c56935341c")


