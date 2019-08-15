; Auto-generated. Do not edit!


(cl:in-package morelab_robot_platform-srv)


;//! \htmlinclude DriveWithSpeed-request.msg.html

(cl:defclass <DriveWithSpeed-request> (roslisp-msg-protocol:ros-message)
  ((left
    :reader left
    :initarg :left
    :type cl:fixnum
    :initform 0)
   (right
    :reader right
    :initarg :right
    :type cl:fixnum
    :initform 0))
)

(cl:defclass DriveWithSpeed-request (<DriveWithSpeed-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DriveWithSpeed-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DriveWithSpeed-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name morelab_robot_platform-srv:<DriveWithSpeed-request> is deprecated: use morelab_robot_platform-srv:DriveWithSpeed-request instead.")))

(cl:ensure-generic-function 'left-val :lambda-list '(m))
(cl:defmethod left-val ((m <DriveWithSpeed-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader morelab_robot_platform-srv:left-val is deprecated.  Use morelab_robot_platform-srv:left instead.")
  (left m))

(cl:ensure-generic-function 'right-val :lambda-list '(m))
(cl:defmethod right-val ((m <DriveWithSpeed-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader morelab_robot_platform-srv:right-val is deprecated.  Use morelab_robot_platform-srv:right instead.")
  (right m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DriveWithSpeed-request>) ostream)
  "Serializes a message object of type '<DriveWithSpeed-request>"
  (cl:let* ((signed (cl:slot-value msg 'left)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'right)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DriveWithSpeed-request>) istream)
  "Deserializes a message object of type '<DriveWithSpeed-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'left) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'right) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DriveWithSpeed-request>)))
  "Returns string type for a service object of type '<DriveWithSpeed-request>"
  "morelab_robot_platform/DriveWithSpeedRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DriveWithSpeed-request)))
  "Returns string type for a service object of type 'DriveWithSpeed-request"
  "morelab_robot_platform/DriveWithSpeedRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DriveWithSpeed-request>)))
  "Returns md5sum for a message object of type '<DriveWithSpeed-request>"
  "09d1b2323a1aeae9343e81809a820b60")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DriveWithSpeed-request)))
  "Returns md5sum for a message object of type 'DriveWithSpeed-request"
  "09d1b2323a1aeae9343e81809a820b60")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DriveWithSpeed-request>)))
  "Returns full string definition for message of type '<DriveWithSpeed-request>"
  (cl:format cl:nil "int16 left~%int16 right~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DriveWithSpeed-request)))
  "Returns full string definition for message of type 'DriveWithSpeed-request"
  (cl:format cl:nil "int16 left~%int16 right~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DriveWithSpeed-request>))
  (cl:+ 0
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DriveWithSpeed-request>))
  "Converts a ROS message object to a list"
  (cl:list 'DriveWithSpeed-request
    (cl:cons ':left (left msg))
    (cl:cons ':right (right msg))
))
;//! \htmlinclude DriveWithSpeed-response.msg.html

(cl:defclass <DriveWithSpeed-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass DriveWithSpeed-response (<DriveWithSpeed-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DriveWithSpeed-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DriveWithSpeed-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name morelab_robot_platform-srv:<DriveWithSpeed-response> is deprecated: use morelab_robot_platform-srv:DriveWithSpeed-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DriveWithSpeed-response>) ostream)
  "Serializes a message object of type '<DriveWithSpeed-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DriveWithSpeed-response>) istream)
  "Deserializes a message object of type '<DriveWithSpeed-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DriveWithSpeed-response>)))
  "Returns string type for a service object of type '<DriveWithSpeed-response>"
  "morelab_robot_platform/DriveWithSpeedResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DriveWithSpeed-response)))
  "Returns string type for a service object of type 'DriveWithSpeed-response"
  "morelab_robot_platform/DriveWithSpeedResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DriveWithSpeed-response>)))
  "Returns md5sum for a message object of type '<DriveWithSpeed-response>"
  "09d1b2323a1aeae9343e81809a820b60")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DriveWithSpeed-response)))
  "Returns md5sum for a message object of type 'DriveWithSpeed-response"
  "09d1b2323a1aeae9343e81809a820b60")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DriveWithSpeed-response>)))
  "Returns full string definition for message of type '<DriveWithSpeed-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DriveWithSpeed-response)))
  "Returns full string definition for message of type 'DriveWithSpeed-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DriveWithSpeed-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DriveWithSpeed-response>))
  "Converts a ROS message object to a list"
  (cl:list 'DriveWithSpeed-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'DriveWithSpeed)))
  'DriveWithSpeed-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'DriveWithSpeed)))
  'DriveWithSpeed-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DriveWithSpeed)))
  "Returns string type for a service object of type '<DriveWithSpeed>"
  "morelab_robot_platform/DriveWithSpeed")