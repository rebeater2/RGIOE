; Auto-generated. Do not edit!


(cl:in-package navplay-msg)


;//! \htmlinclude gnss.msg.html

(cl:defclass <gnss> (roslisp-msg-protocol:ros-message)
  ((gpst
    :reader gpst
    :initarg :gpst
    :type cl:float
    :initform 0.0)
   (lat
    :reader lat
    :initarg :lat
    :type cl:float
    :initform 0.0)
   (lon
    :reader lon
    :initarg :lon
    :type cl:float
    :initform 0.0)
   (h
    :reader h
    :initarg :h
    :type cl:float
    :initform 0.0)
   (pos_std
    :reader pos_std
    :initarg :pos_std
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (mode
    :reader mode
    :initarg :mode
    :type cl:fixnum
    :initform 0)
   (ns
    :reader ns
    :initarg :ns
    :type cl:fixnum
    :initform 0))
)

(cl:defclass gnss (<gnss>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <gnss>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'gnss)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name navplay-msg:<gnss> is deprecated: use navplay-msg:gnss instead.")))

(cl:ensure-generic-function 'gpst-val :lambda-list '(m))
(cl:defmethod gpst-val ((m <gnss>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader navplay-msg:gpst-val is deprecated.  Use navplay-msg:gpst instead.")
  (gpst m))

(cl:ensure-generic-function 'lat-val :lambda-list '(m))
(cl:defmethod lat-val ((m <gnss>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader navplay-msg:lat-val is deprecated.  Use navplay-msg:lat instead.")
  (lat m))

(cl:ensure-generic-function 'lon-val :lambda-list '(m))
(cl:defmethod lon-val ((m <gnss>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader navplay-msg:lon-val is deprecated.  Use navplay-msg:lon instead.")
  (lon m))

(cl:ensure-generic-function 'h-val :lambda-list '(m))
(cl:defmethod h-val ((m <gnss>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader navplay-msg:h-val is deprecated.  Use navplay-msg:h instead.")
  (h m))

(cl:ensure-generic-function 'pos_std-val :lambda-list '(m))
(cl:defmethod pos_std-val ((m <gnss>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader navplay-msg:pos_std-val is deprecated.  Use navplay-msg:pos_std instead.")
  (pos_std m))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <gnss>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader navplay-msg:mode-val is deprecated.  Use navplay-msg:mode instead.")
  (mode m))

(cl:ensure-generic-function 'ns-val :lambda-list '(m))
(cl:defmethod ns-val ((m <gnss>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader navplay-msg:ns-val is deprecated.  Use navplay-msg:ns instead.")
  (ns m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <gnss>) ostream)
  "Serializes a message object of type '<gnss>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'gpst))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'lat))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'lon))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'h))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'pos_std))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ns)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <gnss>) istream)
  "Deserializes a message object of type '<gnss>"
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
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'lat) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'lon) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'h) (roslisp-utils:decode-single-float-bits bits)))
  (cl:setf (cl:slot-value msg 'pos_std) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'pos_std)))
    (cl:dotimes (i 3)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ns)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<gnss>)))
  "Returns string type for a message object of type '<gnss>"
  "navplay/gnss")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gnss)))
  "Returns string type for a message object of type 'gnss"
  "navplay/gnss")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<gnss>)))
  "Returns md5sum for a message object of type '<gnss>"
  "d8b3bc143a1901908ef5f7a6d985bd0b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'gnss)))
  "Returns md5sum for a message object of type 'gnss"
  "d8b3bc143a1901908ef5f7a6d985bd0b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<gnss>)))
  "Returns full string definition for message of type '<gnss>"
  (cl:format cl:nil "float64 gpst~%float64 lat~%float64 lon~%float32 h~%float32[3] pos_std~%uint8 mode~%uint8 ns~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'gnss)))
  "Returns full string definition for message of type 'gnss"
  (cl:format cl:nil "float64 gpst~%float64 lat~%float64 lon~%float32 h~%float32[3] pos_std~%uint8 mode~%uint8 ns~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <gnss>))
  (cl:+ 0
     8
     8
     8
     4
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'pos_std) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <gnss>))
  "Converts a ROS message object to a list"
  (cl:list 'gnss
    (cl:cons ':gpst (gpst msg))
    (cl:cons ':lat (lat msg))
    (cl:cons ':lon (lon msg))
    (cl:cons ':h (h msg))
    (cl:cons ':pos_std (pos_std msg))
    (cl:cons ':mode (mode msg))
    (cl:cons ':ns (ns msg))
))
