; Auto-generated. Do not edit!


(cl:in-package espbot_a-msg)


;//! \htmlinclude Servo_Position.msg.html

(cl:defclass <Servo_Position> (roslisp-msg-protocol:ros-message)
  ((angle
    :reader angle
    :initarg :angle
    :type cl:float
    :initform 0.0))
)

(cl:defclass Servo_Position (<Servo_Position>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Servo_Position>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Servo_Position)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name espbot_a-msg:<Servo_Position> is deprecated: use espbot_a-msg:Servo_Position instead.")))

(cl:ensure-generic-function 'angle-val :lambda-list '(m))
(cl:defmethod angle-val ((m <Servo_Position>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader espbot_a-msg:angle-val is deprecated.  Use espbot_a-msg:angle instead.")
  (angle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Servo_Position>) ostream)
  "Serializes a message object of type '<Servo_Position>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Servo_Position>) istream)
  "Deserializes a message object of type '<Servo_Position>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angle) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Servo_Position>)))
  "Returns string type for a message object of type '<Servo_Position>"
  "espbot_a/Servo_Position")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Servo_Position)))
  "Returns string type for a message object of type 'Servo_Position"
  "espbot_a/Servo_Position")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Servo_Position>)))
  "Returns md5sum for a message object of type '<Servo_Position>"
  "4edb55038e2b888976a0c0c56935341c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Servo_Position)))
  "Returns md5sum for a message object of type 'Servo_Position"
  "4edb55038e2b888976a0c0c56935341c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Servo_Position>)))
  "Returns full string definition for message of type '<Servo_Position>"
  (cl:format cl:nil "float64 angle~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Servo_Position)))
  "Returns full string definition for message of type 'Servo_Position"
  (cl:format cl:nil "float64 angle~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Servo_Position>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Servo_Position>))
  "Converts a ROS message object to a list"
  (cl:list 'Servo_Position
    (cl:cons ':angle (angle msg))
))
