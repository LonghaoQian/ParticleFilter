; Auto-generated. Do not edit!


(cl:in-package keyboard_reader-msg)


;//! \htmlinclude Key.msg.html

(cl:defclass <Key> (roslisp-msg-protocol:ros-message)
  ((key_code
    :reader key_code
    :initarg :key_code
    :type cl:fixnum
    :initform 0)
   (key_name
    :reader key_name
    :initarg :key_name
    :type cl:string
    :initform "")
   (key_pressed
    :reader key_pressed
    :initarg :key_pressed
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Key (<Key>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Key>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Key)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name keyboard_reader-msg:<Key> is deprecated: use keyboard_reader-msg:Key instead.")))

(cl:ensure-generic-function 'key_code-val :lambda-list '(m))
(cl:defmethod key_code-val ((m <Key>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader keyboard_reader-msg:key_code-val is deprecated.  Use keyboard_reader-msg:key_code instead.")
  (key_code m))

(cl:ensure-generic-function 'key_name-val :lambda-list '(m))
(cl:defmethod key_name-val ((m <Key>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader keyboard_reader-msg:key_name-val is deprecated.  Use keyboard_reader-msg:key_name instead.")
  (key_name m))

(cl:ensure-generic-function 'key_pressed-val :lambda-list '(m))
(cl:defmethod key_pressed-val ((m <Key>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader keyboard_reader-msg:key_pressed-val is deprecated.  Use keyboard_reader-msg:key_pressed instead.")
  (key_pressed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Key>) ostream)
  "Serializes a message object of type '<Key>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'key_code)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'key_code)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'key_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'key_name))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'key_pressed) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Key>) istream)
  "Deserializes a message object of type '<Key>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'key_code)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'key_code)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'key_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'key_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'key_pressed) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Key>)))
  "Returns string type for a message object of type '<Key>"
  "keyboard_reader/Key")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Key)))
  "Returns string type for a message object of type 'Key"
  "keyboard_reader/Key")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Key>)))
  "Returns md5sum for a message object of type '<Key>"
  "9709d7232efeba3860fec95e77ac1ae6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Key)))
  "Returns md5sum for a message object of type 'Key"
  "9709d7232efeba3860fec95e77ac1ae6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Key>)))
  "Returns full string definition for message of type '<Key>"
  (cl:format cl:nil "# Key code as defined in linux/inupt.h~%uint16 key_code~%~%# Key name string as defined in evtest, see http://elinux.org/images/9/93/Evtest.c~%string key_name~%~%# 'True' if key was pressed, 'False' otherwise~%bool key_pressed~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Key)))
  "Returns full string definition for message of type 'Key"
  (cl:format cl:nil "# Key code as defined in linux/inupt.h~%uint16 key_code~%~%# Key name string as defined in evtest, see http://elinux.org/images/9/93/Evtest.c~%string key_name~%~%# 'True' if key was pressed, 'False' otherwise~%bool key_pressed~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Key>))
  (cl:+ 0
     2
     4 (cl:length (cl:slot-value msg 'key_name))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Key>))
  "Converts a ROS message object to a list"
  (cl:list 'Key
    (cl:cons ':key_code (key_code msg))
    (cl:cons ':key_name (key_name msg))
    (cl:cons ':key_pressed (key_pressed msg))
))
