; Auto-generated. Do not edit!


(cl:in-package morelab_robot_platform-srv)


;//! \htmlinclude GetHeading-request.msg.html

(cl:defclass <GetHeading-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetHeading-request (<GetHeading-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetHeading-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetHeading-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name morelab_robot_platform-srv:<GetHeading-request> is deprecated: use morelab_robot_platform-srv:GetHeading-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetHeading-request>) ostream)
  "Serializes a message object of type '<GetHeading-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetHeading-request>) istream)
  "Deserializes a message object of type '<GetHeading-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetHeading-request>)))
  "Returns string type for a service object of type '<GetHeading-request>"
  "morelab_robot_platform/GetHeadingRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetHeading-request)))
  "Returns string type for a service object of type 'GetHeading-request"
  "morelab_robot_platform/GetHeadingRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetHeading-request>)))
  "Returns md5sum for a message object of type '<GetHeading-request>"
  "3204bea69e84bbd63420c608a176827e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetHeading-request)))
  "Returns md5sum for a message object of type 'GetHeading-request"
  "3204bea69e84bbd63420c608a176827e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetHeading-request>)))
  "Returns full string definition for message of type '<GetHeading-request>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetHeading-request)))
  "Returns full string definition for message of type 'GetHeading-request"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetHeading-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetHeading-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetHeading-request
))
;//! \htmlinclude GetHeading-response.msg.html

(cl:defclass <GetHeading-response> (roslisp-msg-protocol:ros-message)
  ((heading
    :reader heading
    :initarg :heading
    :type cl:fixnum
    :initform 0))
)

(cl:defclass GetHeading-response (<GetHeading-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetHeading-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetHeading-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name morelab_robot_platform-srv:<GetHeading-response> is deprecated: use morelab_robot_platform-srv:GetHeading-response instead.")))

(cl:ensure-generic-function 'heading-val :lambda-list '(m))
(cl:defmethod heading-val ((m <GetHeading-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader morelab_robot_platform-srv:heading-val is deprecated.  Use morelab_robot_platform-srv:heading instead.")
  (heading m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetHeading-response>) ostream)
  "Serializes a message object of type '<GetHeading-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'heading)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'heading)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetHeading-response>) istream)
  "Deserializes a message object of type '<GetHeading-response>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'heading)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'heading)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetHeading-response>)))
  "Returns string type for a service object of type '<GetHeading-response>"
  "morelab_robot_platform/GetHeadingResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetHeading-response)))
  "Returns string type for a service object of type 'GetHeading-response"
  "morelab_robot_platform/GetHeadingResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetHeading-response>)))
  "Returns md5sum for a message object of type '<GetHeading-response>"
  "3204bea69e84bbd63420c608a176827e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetHeading-response)))
  "Returns md5sum for a message object of type 'GetHeading-response"
  "3204bea69e84bbd63420c608a176827e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetHeading-response>)))
  "Returns full string definition for message of type '<GetHeading-response>"
  (cl:format cl:nil "uint16 heading~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetHeading-response)))
  "Returns full string definition for message of type 'GetHeading-response"
  (cl:format cl:nil "uint16 heading~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetHeading-response>))
  (cl:+ 0
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetHeading-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetHeading-response
    (cl:cons ':heading (heading msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetHeading)))
  'GetHeading-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetHeading)))
  'GetHeading-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetHeading)))
  "Returns string type for a service object of type '<GetHeading>"
  "morelab_robot_platform/GetHeading")