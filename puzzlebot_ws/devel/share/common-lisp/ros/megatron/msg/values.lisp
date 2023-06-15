; Auto-generated. Do not edit!


(cl:in-package megatron-msg)


;//! \htmlinclude values.msg.html

(cl:defclass <values> (roslisp-msg-protocol:ros-message)
  ((direction
    :reader direction
    :initarg :direction
    :type cl:string
    :initform "")
   (velocity
    :reader velocity
    :initarg :velocity
    :type cl:string
    :initform ""))
)

(cl:defclass values (<values>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <values>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'values)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name megatron-msg:<values> is deprecated: use megatron-msg:values instead.")))

(cl:ensure-generic-function 'direction-val :lambda-list '(m))
(cl:defmethod direction-val ((m <values>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader megatron-msg:direction-val is deprecated.  Use megatron-msg:direction instead.")
  (direction m))

(cl:ensure-generic-function 'velocity-val :lambda-list '(m))
(cl:defmethod velocity-val ((m <values>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader megatron-msg:velocity-val is deprecated.  Use megatron-msg:velocity instead.")
  (velocity m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <values>) ostream)
  "Serializes a message object of type '<values>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'direction))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'direction))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'velocity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'velocity))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <values>) istream)
  "Deserializes a message object of type '<values>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'direction) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'direction) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'velocity) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'velocity) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<values>)))
  "Returns string type for a message object of type '<values>"
  "megatron/values")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'values)))
  "Returns string type for a message object of type 'values"
  "megatron/values")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<values>)))
  "Returns md5sum for a message object of type '<values>"
  "4cf332b5242ea2dc6436208b418b4da2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'values)))
  "Returns md5sum for a message object of type 'values"
  "4cf332b5242ea2dc6436208b418b4da2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<values>)))
  "Returns full string definition for message of type '<values>"
  (cl:format cl:nil "string direction~%string velocity~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'values)))
  "Returns full string definition for message of type 'values"
  (cl:format cl:nil "string direction~%string velocity~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <values>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'direction))
     4 (cl:length (cl:slot-value msg 'velocity))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <values>))
  "Converts a ROS message object to a list"
  (cl:list 'values
    (cl:cons ':direction (direction msg))
    (cl:cons ':velocity (velocity msg))
))
