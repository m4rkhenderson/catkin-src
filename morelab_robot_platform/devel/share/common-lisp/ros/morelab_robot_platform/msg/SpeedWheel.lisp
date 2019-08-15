; Auto-generated. Do not edit!


(cl:in-package morelab_robot_platform-msg)


;//! \htmlinclude SpeedWheel.msg.html

(cl:defclass <SpeedWheel> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:string
    :initform "")
   (value
    :reader value
    :initarg :value
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass SpeedWheel (<SpeedWheel>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SpeedWheel>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SpeedWheel)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name morelab_robot_platform-msg:<SpeedWheel> is deprecated: use morelab_robot_platform-msg:SpeedWheel instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <SpeedWheel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader morelab_robot_platform-msg:status-val is deprecated.  Use morelab_robot_platform-msg:status instead.")
  (status m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <SpeedWheel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader morelab_robot_platform-msg:value-val is deprecated.  Use morelab_robot_platform-msg:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SpeedWheel>) ostream)
  "Serializes a message object of type '<SpeedWheel>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'status))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'status))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    ))
   (cl:slot-value msg 'value))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SpeedWheel>) istream)
  "Deserializes a message object of type '<SpeedWheel>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'status) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'status) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'value) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'value)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SpeedWheel>)))
  "Returns string type for a message object of type '<SpeedWheel>"
  "morelab_robot_platform/SpeedWheel")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SpeedWheel)))
  "Returns string type for a message object of type 'SpeedWheel"
  "morelab_robot_platform/SpeedWheel")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SpeedWheel>)))
  "Returns md5sum for a message object of type '<SpeedWheel>"
  "ad3e9cd6d6636b6beca6cffe0be63ef6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SpeedWheel)))
  "Returns md5sum for a message object of type 'SpeedWheel"
  "ad3e9cd6d6636b6beca6cffe0be63ef6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SpeedWheel>)))
  "Returns full string definition for message of type '<SpeedWheel>"
  (cl:format cl:nil "string status~%int16[] value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SpeedWheel)))
  "Returns full string definition for message of type 'SpeedWheel"
  (cl:format cl:nil "string status~%int16[] value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SpeedWheel>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'status))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'value) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SpeedWheel>))
  "Converts a ROS message object to a list"
  (cl:list 'SpeedWheel
    (cl:cons ':status (status msg))
    (cl:cons ':value (value msg))
))
