; Auto-generated. Do not edit!


(cl:in-package navplay-msg)


;//! \htmlinclude vel.msg.html

(cl:defclass <vel> (roslisp-msg-protocol:ros-message)
  ((gpst
    :reader gpst
    :initarg :gpst
    :type cl:float
    :initform 0.0)
   (forward
    :reader forward
    :initarg :forward
    :type cl:float
    :initform 0.0)
   (angular
    :reader angular
    :initarg :angular
    :type cl:float
    :initform 0.0))
)

(cl:defclass vel (<vel>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <vel>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'vel)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name navplay-msg:<vel> is deprecated: use navplay-msg:vel instead.")))

(cl:ensure-generic-function 'gpst-val :lambda-list '(m))
(cl:defmethod gpst-val ((m <vel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader navplay-msg:gpst-val is deprecated.  Use navplay-msg:gpst instead.")
  (gpst m))

(cl:ensure-generic-function 'forward-val :lambda-list '(m))
(cl:defmethod forward-val ((m <vel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader navplay-msg:forward-val is deprecated.  Use navplay-msg:forward instead.")
  (forward m))

(cl:ensure-generic-function 'angular-val :lambda-list '(m))
(cl:defmethod angular-val ((m <vel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader navplay-msg:angular-val is deprecated.  Use navplay-msg:angular instead.")
  (angular m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <vel>) ostream)
  "Serializes a message object of type '<vel>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'gpst))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'forward))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'angular))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <vel>) istream)
  "Deserializes a message object of type '<vel>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'gpst) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'forward) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angular) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<vel>)))
  "Returns string type for a message object of type '<vel>"
  "navplay/vel")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'vel)))
  "Returns string type for a message object of type 'vel"
  "navplay/vel")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<vel>)))
  "Returns md5sum for a message object of type '<vel>"
  "7e11ee5e5f9a364893511f8eb7b4a576")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'vel)))
  "Returns md5sum for a message object of type 'vel"
  "7e11ee5e5f9a364893511f8eb7b4a576")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<vel>)))
  "Returns full string definition for message of type '<vel>"
  (cl:format cl:nil "float64 gpst~%float32 forward~%float32 angular~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'vel)))
  "Returns full string definition for message of type 'vel"
  (cl:format cl:nil "float64 gpst~%float32 forward~%float32 angular~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <vel>))
  (cl:+ 0
     8
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <vel>))
  "Converts a ROS message object to a list"
  (cl:list 'vel
    (cl:cons ':gpst (gpst msg))
    (cl:cons ':forward (forward msg))
    (cl:cons ':angular (angular msg))
))
