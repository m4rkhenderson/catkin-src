; Auto-generated. Do not edit!


(cl:in-package morelab_robot_platform-srv)


;//! \htmlinclude GetStatus-request.msg.html

(cl:defclass <GetStatus-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetStatus-request (<GetStatus-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetStatus-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetStatus-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name morelab_robot_platform-srv:<GetStatus-request> is deprecated: use morelab_robot_platform-srv:GetStatus-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetStatus-request>) ostream)
  "Serializes a message object of type '<GetStatus-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetStatus-request>) istream)
  "Deserializes a message object of type '<GetStatus-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetStatus-request>)))
  "Returns string type for a service object of type '<GetStatus-request>"
  "morelab_robot_platform/GetStatusRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetStatus-request)))
  "Returns string type for a service object of type 'GetStatus-request"
  "morelab_robot_platform/GetStatusRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetStatus-request>)))
  "Returns md5sum for a message object of type '<GetStatus-request>"
  "1d68d83766b794b474930687da7dce82")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetStatus-request)))
  "Returns md5sum for a message object of type 'GetStatus-request"
  "1d68d83766b794b474930687da7dce82")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetStatus-request>)))
  "Returns full string definition for message of type '<GetStatus-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetStatus-request)))
  "Returns full string definition for message of type 'GetStatus-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetStatus-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetStatus-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetStatus-request
))
;//! \htmlinclude GetStatus-response.msg.html

(cl:defclass <GetStatus-response> (roslisp-msg-protocol:ros-message)
  ((okay
    :reader okay
    :initarg :okay
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass GetStatus-response (<GetStatus-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetStatus-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetStatus-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name morelab_robot_platform-srv:<GetStatus-response> is deprecated: use morelab_robot_platform-srv:GetStatus-response instead.")))

(cl:ensure-generic-function 'okay-val :lambda-list '(m))
(cl:defmethod okay-val ((m <GetStatus-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader morelab_robot_platform-srv:okay-val is deprecated.  Use morelab_robot_platform-srv:okay instead.")
  (okay m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetStatus-response>) ostream)
  "Serializes a message object of type '<GetStatus-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'okay) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetStatus-response>) istream)
  "Deserializes a message object of type '<GetStatus-response>"
    (cl:setf (cl:slot-value msg 'okay) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetStatus-response>)))
  "Returns string type for a service object of type '<GetStatus-response>"
  "morelab_robot_platform/GetStatusResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetStatus-response)))
  "Returns string type for a service object of type 'GetStatus-response"
  "morelab_robot_platform/GetStatusResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetStatus-response>)))
  "Returns md5sum for a message object of type '<GetStatus-response>"
  "1d68d83766b794b474930687da7dce82")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetStatus-response)))
  "Returns md5sum for a message object of type 'GetStatus-response"
  "1d68d83766b794b474930687da7dce82")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetStatus-response>)))
  "Returns full string definition for message of type '<GetStatus-response>"
  (cl:format cl:nil "bool okay~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetStatus-response)))
  "Returns full string definition for message of type 'GetStatus-response"
  (cl:format cl:nil "bool okay~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetStatus-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetStatus-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetStatus-response
    (cl:cons ':okay (okay msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetStatus)))
  'GetStatus-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetStatus)))
  'GetStatus-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetStatus)))
  "Returns string type for a service object of type '<GetStatus>"
  "morelab_robot_platform/GetStatus")