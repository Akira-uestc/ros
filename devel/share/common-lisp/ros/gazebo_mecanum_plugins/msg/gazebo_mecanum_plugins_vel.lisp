; Auto-generated. Do not edit!


(cl:in-package gazebo_mecanum_plugins-msg)


;//! \htmlinclude gazebo_mecanum_plugins_vel.msg.html

(cl:defclass <gazebo_mecanum_plugins_vel> (roslisp-msg-protocol:ros-message)
  ((LEFT_REAR_vel
    :reader LEFT_REAR_vel
    :initarg :LEFT_REAR_vel
    :type cl:float
    :initform 0.0)
   (LEFT_FRONT_vel
    :reader LEFT_FRONT_vel
    :initarg :LEFT_FRONT_vel
    :type cl:float
    :initform 0.0)
   (RIGHT_FRONT_vel
    :reader RIGHT_FRONT_vel
    :initarg :RIGHT_FRONT_vel
    :type cl:float
    :initform 0.0)
   (RIGHT_REAR_vel
    :reader RIGHT_REAR_vel
    :initarg :RIGHT_REAR_vel
    :type cl:float
    :initform 0.0))
)

(cl:defclass gazebo_mecanum_plugins_vel (<gazebo_mecanum_plugins_vel>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <gazebo_mecanum_plugins_vel>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'gazebo_mecanum_plugins_vel)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name gazebo_mecanum_plugins-msg:<gazebo_mecanum_plugins_vel> is deprecated: use gazebo_mecanum_plugins-msg:gazebo_mecanum_plugins_vel instead.")))

(cl:ensure-generic-function 'LEFT_REAR_vel-val :lambda-list '(m))
(cl:defmethod LEFT_REAR_vel-val ((m <gazebo_mecanum_plugins_vel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gazebo_mecanum_plugins-msg:LEFT_REAR_vel-val is deprecated.  Use gazebo_mecanum_plugins-msg:LEFT_REAR_vel instead.")
  (LEFT_REAR_vel m))

(cl:ensure-generic-function 'LEFT_FRONT_vel-val :lambda-list '(m))
(cl:defmethod LEFT_FRONT_vel-val ((m <gazebo_mecanum_plugins_vel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gazebo_mecanum_plugins-msg:LEFT_FRONT_vel-val is deprecated.  Use gazebo_mecanum_plugins-msg:LEFT_FRONT_vel instead.")
  (LEFT_FRONT_vel m))

(cl:ensure-generic-function 'RIGHT_FRONT_vel-val :lambda-list '(m))
(cl:defmethod RIGHT_FRONT_vel-val ((m <gazebo_mecanum_plugins_vel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gazebo_mecanum_plugins-msg:RIGHT_FRONT_vel-val is deprecated.  Use gazebo_mecanum_plugins-msg:RIGHT_FRONT_vel instead.")
  (RIGHT_FRONT_vel m))

(cl:ensure-generic-function 'RIGHT_REAR_vel-val :lambda-list '(m))
(cl:defmethod RIGHT_REAR_vel-val ((m <gazebo_mecanum_plugins_vel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gazebo_mecanum_plugins-msg:RIGHT_REAR_vel-val is deprecated.  Use gazebo_mecanum_plugins-msg:RIGHT_REAR_vel instead.")
  (RIGHT_REAR_vel m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <gazebo_mecanum_plugins_vel>) ostream)
  "Serializes a message object of type '<gazebo_mecanum_plugins_vel>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'LEFT_REAR_vel))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'LEFT_FRONT_vel))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'RIGHT_FRONT_vel))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'RIGHT_REAR_vel))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <gazebo_mecanum_plugins_vel>) istream)
  "Deserializes a message object of type '<gazebo_mecanum_plugins_vel>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'LEFT_REAR_vel) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'LEFT_FRONT_vel) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'RIGHT_FRONT_vel) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'RIGHT_REAR_vel) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<gazebo_mecanum_plugins_vel>)))
  "Returns string type for a message object of type '<gazebo_mecanum_plugins_vel>"
  "gazebo_mecanum_plugins/gazebo_mecanum_plugins_vel")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gazebo_mecanum_plugins_vel)))
  "Returns string type for a message object of type 'gazebo_mecanum_plugins_vel"
  "gazebo_mecanum_plugins/gazebo_mecanum_plugins_vel")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<gazebo_mecanum_plugins_vel>)))
  "Returns md5sum for a message object of type '<gazebo_mecanum_plugins_vel>"
  "f9df474704d7113e588f9ced0981881c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'gazebo_mecanum_plugins_vel)))
  "Returns md5sum for a message object of type 'gazebo_mecanum_plugins_vel"
  "f9df474704d7113e588f9ced0981881c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<gazebo_mecanum_plugins_vel>)))
  "Returns full string definition for message of type '<gazebo_mecanum_plugins_vel>"
  (cl:format cl:nil "float64 LEFT_REAR_vel~%float64 LEFT_FRONT_vel~%float64 RIGHT_FRONT_vel~%float64 RIGHT_REAR_vel~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'gazebo_mecanum_plugins_vel)))
  "Returns full string definition for message of type 'gazebo_mecanum_plugins_vel"
  (cl:format cl:nil "float64 LEFT_REAR_vel~%float64 LEFT_FRONT_vel~%float64 RIGHT_FRONT_vel~%float64 RIGHT_REAR_vel~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <gazebo_mecanum_plugins_vel>))
  (cl:+ 0
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <gazebo_mecanum_plugins_vel>))
  "Converts a ROS message object to a list"
  (cl:list 'gazebo_mecanum_plugins_vel
    (cl:cons ':LEFT_REAR_vel (LEFT_REAR_vel msg))
    (cl:cons ':LEFT_FRONT_vel (LEFT_FRONT_vel msg))
    (cl:cons ':RIGHT_FRONT_vel (RIGHT_FRONT_vel msg))
    (cl:cons ':RIGHT_REAR_vel (RIGHT_REAR_vel msg))
))
