; Auto-generated. Do not edit!


(cl:in-package morelab_robot_platform-msg)


;//! \htmlinclude DistanceWheel.msg.html

(cl:defclass <DistanceWheel> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:string
    :initform "")
   (value
    :reader value
    :initarg :value
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass DistanceWheel (<DistanceWheel>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DistanceWheel>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DistanceWheel)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name morelab_robot_platform-msg:<DistanceWheel> is deprecated: use morelab_robot_platform-msg:DistanceWheel instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <DistanceWheel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader morelab_robot_platform-msg:status-val is deprecated.  Use morelab_robot_platform-msg:status instead.")
  (status m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <DistanceWheel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader morelab_robot_platform-msg:value-val is deprecated.  Use morelab_robot_platform-msg:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DistanceWheel>) ostream)
  "Serializes a message object of type '<DistanceWheel>"
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
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'value))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DistanceWheel>) istream)
  "Deserializes a message object of type '<DistanceWheel>"
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
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DistanceWheel>)))
  "Returns string type for a message object of type '<DistanceWheel>"
  "morelab_robot_platform/DistanceWheel")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DistanceWheel)))
  "Returns string type for a message object of type 'DistanceWheel"
  "morelab_robot_platform/DistanceWheel")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DistanceWheel>)))
  "Returns md5sum for a message object of type '<DistanceWheel>"
  "be759ba118a1f0b9bfe13076a2ab59f7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DistanceWheel)))
  "Returns md5sum for a message object of type 'DistanceWheel"
  "be759ba118a1f0b9bfe13076a2ab59f7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DistanceWheel>)))
  "Returns full string definition for message of type '<DistanceWheel>"
  (cl:format cl:nil "string status~%int32[] value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DistanceWheel)))
  "Returns full string definition for message of type 'DistanceWheel"
  (cl:format cl:nil "string status~%int32[] value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DistanceWheel>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'status))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'value) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DistanceWheel>))
  "Converts a ROS message object to a list"
  (cl:list 'DistanceWheel
    (cl:cons ':status (status msg))
    (cl:cons ':value (value msg))
))
