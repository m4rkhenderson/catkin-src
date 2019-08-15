; Auto-generated. Do not edit!


(cl:in-package morelab_robot_platform-msg)


;//! \htmlinclude StopSlidingSignal.msg.html

(cl:defclass <StopSlidingSignal> (roslisp-msg-protocol:ros-message)
  ((value
    :reader value
    :initarg :value
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass StopSlidingSignal (<StopSlidingSignal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StopSlidingSignal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StopSlidingSignal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name morelab_robot_platform-msg:<StopSlidingSignal> is deprecated: use morelab_robot_platform-msg:StopSlidingSignal instead.")))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <StopSlidingSignal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader morelab_robot_platform-msg:value-val is deprecated.  Use morelab_robot_platform-msg:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StopSlidingSignal>) ostream)
  "Serializes a message object of type '<StopSlidingSignal>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'value) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StopSlidingSignal>) istream)
  "Deserializes a message object of type '<StopSlidingSignal>"
    (cl:setf (cl:slot-value msg 'value) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StopSlidingSignal>)))
  "Returns string type for a message object of type '<StopSlidingSignal>"
  "morelab_robot_platform/StopSlidingSignal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StopSlidingSignal)))
  "Returns string type for a message object of type 'StopSlidingSignal"
  "morelab_robot_platform/StopSlidingSignal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StopSlidingSignal>)))
  "Returns md5sum for a message object of type '<StopSlidingSignal>"
  "e431d687bf4b2c65fbd94b12ae0cb5d9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StopSlidingSignal)))
  "Returns md5sum for a message object of type 'StopSlidingSignal"
  "e431d687bf4b2c65fbd94b12ae0cb5d9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StopSlidingSignal>)))
  "Returns full string definition for message of type '<StopSlidingSignal>"
  (cl:format cl:nil "bool value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StopSlidingSignal)))
  "Returns full string definition for message of type 'StopSlidingSignal"
  (cl:format cl:nil "bool value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StopSlidingSignal>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StopSlidingSignal>))
  "Converts a ROS message object to a list"
  (cl:list 'StopSlidingSignal
    (cl:cons ':value (value msg))
))
