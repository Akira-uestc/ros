; Auto-generated. Do not edit!


(cl:in-package gazebo_mecanum_plugins-msg)


;//! \htmlinclude gazebo_mecanum_plugins_pid.msg.html

(cl:defclass <gazebo_mecanum_plugins_pid> (roslisp-msg-protocol:ros-message)
  ((LR_P
    :reader LR_P
    :initarg :LR_P
    :type cl:float
    :initform 0.0)
   (LR_I
    :reader LR_I
    :initarg :LR_I
    :type cl:float
    :initform 0.0)
   (LR_D
    :reader LR_D
    :initarg :LR_D
    :type cl:float
    :initform 0.0)
   (LF_P
    :reader LF_P
    :initarg :LF_P
    :type cl:float
    :initform 0.0)
   (LF_I
    :reader LF_I
    :initarg :LF_I
    :type cl:float
    :initform 0.0)
   (LF_D
    :reader LF_D
    :initarg :LF_D
    :type cl:float
    :initform 0.0)
   (RF_P
    :reader RF_P
    :initarg :RF_P
    :type cl:float
    :initform 0.0)
   (RF_I
    :reader RF_I
    :initarg :RF_I
    :type cl:float
    :initform 0.0)
   (RF_D
    :reader RF_D
    :initarg :RF_D
    :type cl:float
    :initform 0.0)
   (RR_P
    :reader RR_P
    :initarg :RR_P
    :type cl:float
    :initform 0.0)
   (RR_I
    :reader RR_I
    :initarg :RR_I
    :type cl:float
    :initform 0.0)
   (RR_D
    :reader RR_D
    :initarg :RR_D
    :type cl:float
    :initform 0.0))
)

(cl:defclass gazebo_mecanum_plugins_pid (<gazebo_mecanum_plugins_pid>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <gazebo_mecanum_plugins_pid>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'gazebo_mecanum_plugins_pid)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name gazebo_mecanum_plugins-msg:<gazebo_mecanum_plugins_pid> is deprecated: use gazebo_mecanum_plugins-msg:gazebo_mecanum_plugins_pid instead.")))

(cl:ensure-generic-function 'LR_P-val :lambda-list '(m))
(cl:defmethod LR_P-val ((m <gazebo_mecanum_plugins_pid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gazebo_mecanum_plugins-msg:LR_P-val is deprecated.  Use gazebo_mecanum_plugins-msg:LR_P instead.")
  (LR_P m))

(cl:ensure-generic-function 'LR_I-val :lambda-list '(m))
(cl:defmethod LR_I-val ((m <gazebo_mecanum_plugins_pid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gazebo_mecanum_plugins-msg:LR_I-val is deprecated.  Use gazebo_mecanum_plugins-msg:LR_I instead.")
  (LR_I m))

(cl:ensure-generic-function 'LR_D-val :lambda-list '(m))
(cl:defmethod LR_D-val ((m <gazebo_mecanum_plugins_pid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gazebo_mecanum_plugins-msg:LR_D-val is deprecated.  Use gazebo_mecanum_plugins-msg:LR_D instead.")
  (LR_D m))

(cl:ensure-generic-function 'LF_P-val :lambda-list '(m))
(cl:defmethod LF_P-val ((m <gazebo_mecanum_plugins_pid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gazebo_mecanum_plugins-msg:LF_P-val is deprecated.  Use gazebo_mecanum_plugins-msg:LF_P instead.")
  (LF_P m))

(cl:ensure-generic-function 'LF_I-val :lambda-list '(m))
(cl:defmethod LF_I-val ((m <gazebo_mecanum_plugins_pid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gazebo_mecanum_plugins-msg:LF_I-val is deprecated.  Use gazebo_mecanum_plugins-msg:LF_I instead.")
  (LF_I m))

(cl:ensure-generic-function 'LF_D-val :lambda-list '(m))
(cl:defmethod LF_D-val ((m <gazebo_mecanum_plugins_pid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gazebo_mecanum_plugins-msg:LF_D-val is deprecated.  Use gazebo_mecanum_plugins-msg:LF_D instead.")
  (LF_D m))

(cl:ensure-generic-function 'RF_P-val :lambda-list '(m))
(cl:defmethod RF_P-val ((m <gazebo_mecanum_plugins_pid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gazebo_mecanum_plugins-msg:RF_P-val is deprecated.  Use gazebo_mecanum_plugins-msg:RF_P instead.")
  (RF_P m))

(cl:ensure-generic-function 'RF_I-val :lambda-list '(m))
(cl:defmethod RF_I-val ((m <gazebo_mecanum_plugins_pid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gazebo_mecanum_plugins-msg:RF_I-val is deprecated.  Use gazebo_mecanum_plugins-msg:RF_I instead.")
  (RF_I m))

(cl:ensure-generic-function 'RF_D-val :lambda-list '(m))
(cl:defmethod RF_D-val ((m <gazebo_mecanum_plugins_pid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gazebo_mecanum_plugins-msg:RF_D-val is deprecated.  Use gazebo_mecanum_plugins-msg:RF_D instead.")
  (RF_D m))

(cl:ensure-generic-function 'RR_P-val :lambda-list '(m))
(cl:defmethod RR_P-val ((m <gazebo_mecanum_plugins_pid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gazebo_mecanum_plugins-msg:RR_P-val is deprecated.  Use gazebo_mecanum_plugins-msg:RR_P instead.")
  (RR_P m))

(cl:ensure-generic-function 'RR_I-val :lambda-list '(m))
(cl:defmethod RR_I-val ((m <gazebo_mecanum_plugins_pid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gazebo_mecanum_plugins-msg:RR_I-val is deprecated.  Use gazebo_mecanum_plugins-msg:RR_I instead.")
  (RR_I m))

(cl:ensure-generic-function 'RR_D-val :lambda-list '(m))
(cl:defmethod RR_D-val ((m <gazebo_mecanum_plugins_pid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gazebo_mecanum_plugins-msg:RR_D-val is deprecated.  Use gazebo_mecanum_plugins-msg:RR_D instead.")
  (RR_D m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <gazebo_mecanum_plugins_pid>) ostream)
  "Serializes a message object of type '<gazebo_mecanum_plugins_pid>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'LR_P))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'LR_I))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'LR_D))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'LF_P))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'LF_I))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'LF_D))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'RF_P))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'RF_I))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'RF_D))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'RR_P))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'RR_I))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'RR_D))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <gazebo_mecanum_plugins_pid>) istream)
  "Deserializes a message object of type '<gazebo_mecanum_plugins_pid>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'LR_P) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'LR_I) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'LR_D) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'LF_P) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'LF_I) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'LF_D) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'RF_P) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'RF_I) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'RF_D) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'RR_P) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'RR_I) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'RR_D) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<gazebo_mecanum_plugins_pid>)))
  "Returns string type for a message object of type '<gazebo_mecanum_plugins_pid>"
  "gazebo_mecanum_plugins/gazebo_mecanum_plugins_pid")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gazebo_mecanum_plugins_pid)))
  "Returns string type for a message object of type 'gazebo_mecanum_plugins_pid"
  "gazebo_mecanum_plugins/gazebo_mecanum_plugins_pid")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<gazebo_mecanum_plugins_pid>)))
  "Returns md5sum for a message object of type '<gazebo_mecanum_plugins_pid>"
  "9eb89efea79b06cdde55828e8174754b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'gazebo_mecanum_plugins_pid)))
  "Returns md5sum for a message object of type 'gazebo_mecanum_plugins_pid"
  "9eb89efea79b06cdde55828e8174754b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<gazebo_mecanum_plugins_pid>)))
  "Returns full string definition for message of type '<gazebo_mecanum_plugins_pid>"
  (cl:format cl:nil "float64 LR_P~%float64 LR_I~%float64 LR_D~%float64 LF_P~%float64 LF_I~%float64 LF_D~%float64 RF_P~%float64 RF_I~%float64 RF_D~%float64 RR_P~%float64 RR_I~%float64 RR_D~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'gazebo_mecanum_plugins_pid)))
  "Returns full string definition for message of type 'gazebo_mecanum_plugins_pid"
  (cl:format cl:nil "float64 LR_P~%float64 LR_I~%float64 LR_D~%float64 LF_P~%float64 LF_I~%float64 LF_D~%float64 RF_P~%float64 RF_I~%float64 RF_D~%float64 RR_P~%float64 RR_I~%float64 RR_D~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <gazebo_mecanum_plugins_pid>))
  (cl:+ 0
     8
     8
     8
     8
     8
     8
     8
     8
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <gazebo_mecanum_plugins_pid>))
  "Converts a ROS message object to a list"
  (cl:list 'gazebo_mecanum_plugins_pid
    (cl:cons ':LR_P (LR_P msg))
    (cl:cons ':LR_I (LR_I msg))
    (cl:cons ':LR_D (LR_D msg))
    (cl:cons ':LF_P (LF_P msg))
    (cl:cons ':LF_I (LF_I msg))
    (cl:cons ':LF_D (LF_D msg))
    (cl:cons ':RF_P (RF_P msg))
    (cl:cons ':RF_I (RF_I msg))
    (cl:cons ':RF_D (RF_D msg))
    (cl:cons ':RR_P (RR_P msg))
    (cl:cons ':RR_I (RR_I msg))
    (cl:cons ':RR_D (RR_D msg))
))
