;; Auto-generated. Do not edit!


(when (boundp 'gazebo_mecanum_plugins::gazebo_mecanum_plugins_vel)
  (if (not (find-package "GAZEBO_MECANUM_PLUGINS"))
    (make-package "GAZEBO_MECANUM_PLUGINS"))
  (shadow 'gazebo_mecanum_plugins_vel (find-package "GAZEBO_MECANUM_PLUGINS")))
(unless (find-package "GAZEBO_MECANUM_PLUGINS::GAZEBO_MECANUM_PLUGINS_VEL")
  (make-package "GAZEBO_MECANUM_PLUGINS::GAZEBO_MECANUM_PLUGINS_VEL"))

(in-package "ROS")
;;//! \htmlinclude gazebo_mecanum_plugins_vel.msg.html


(defclass gazebo_mecanum_plugins::gazebo_mecanum_plugins_vel
  :super ros::object
  :slots (_LEFT_REAR_vel _LEFT_FRONT_vel _RIGHT_FRONT_vel _RIGHT_REAR_vel ))

(defmethod gazebo_mecanum_plugins::gazebo_mecanum_plugins_vel
  (:init
   (&key
    ((:LEFT_REAR_vel __LEFT_REAR_vel) 0.0)
    ((:LEFT_FRONT_vel __LEFT_FRONT_vel) 0.0)
    ((:RIGHT_FRONT_vel __RIGHT_FRONT_vel) 0.0)
    ((:RIGHT_REAR_vel __RIGHT_REAR_vel) 0.0)
    )
   (send-super :init)
   (setq _LEFT_REAR_vel (float __LEFT_REAR_vel))
   (setq _LEFT_FRONT_vel (float __LEFT_FRONT_vel))
   (setq _RIGHT_FRONT_vel (float __RIGHT_FRONT_vel))
   (setq _RIGHT_REAR_vel (float __RIGHT_REAR_vel))
   self)
  (:LEFT_REAR_vel
   (&optional __LEFT_REAR_vel)
   (if __LEFT_REAR_vel (setq _LEFT_REAR_vel __LEFT_REAR_vel)) _LEFT_REAR_vel)
  (:LEFT_FRONT_vel
   (&optional __LEFT_FRONT_vel)
   (if __LEFT_FRONT_vel (setq _LEFT_FRONT_vel __LEFT_FRONT_vel)) _LEFT_FRONT_vel)
  (:RIGHT_FRONT_vel
   (&optional __RIGHT_FRONT_vel)
   (if __RIGHT_FRONT_vel (setq _RIGHT_FRONT_vel __RIGHT_FRONT_vel)) _RIGHT_FRONT_vel)
  (:RIGHT_REAR_vel
   (&optional __RIGHT_REAR_vel)
   (if __RIGHT_REAR_vel (setq _RIGHT_REAR_vel __RIGHT_REAR_vel)) _RIGHT_REAR_vel)
  (:serialization-length
   ()
   (+
    ;; float64 _LEFT_REAR_vel
    8
    ;; float64 _LEFT_FRONT_vel
    8
    ;; float64 _RIGHT_FRONT_vel
    8
    ;; float64 _RIGHT_REAR_vel
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _LEFT_REAR_vel
       (sys::poke _LEFT_REAR_vel (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _LEFT_FRONT_vel
       (sys::poke _LEFT_FRONT_vel (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _RIGHT_FRONT_vel
       (sys::poke _RIGHT_FRONT_vel (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _RIGHT_REAR_vel
       (sys::poke _RIGHT_REAR_vel (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _LEFT_REAR_vel
     (setq _LEFT_REAR_vel (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _LEFT_FRONT_vel
     (setq _LEFT_FRONT_vel (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _RIGHT_FRONT_vel
     (setq _RIGHT_FRONT_vel (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _RIGHT_REAR_vel
     (setq _RIGHT_REAR_vel (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get gazebo_mecanum_plugins::gazebo_mecanum_plugins_vel :md5sum-) "f9df474704d7113e588f9ced0981881c")
(setf (get gazebo_mecanum_plugins::gazebo_mecanum_plugins_vel :datatype-) "gazebo_mecanum_plugins/gazebo_mecanum_plugins_vel")
(setf (get gazebo_mecanum_plugins::gazebo_mecanum_plugins_vel :definition-)
      "float64 LEFT_REAR_vel
float64 LEFT_FRONT_vel
float64 RIGHT_FRONT_vel
float64 RIGHT_REAR_vel
")



(provide :gazebo_mecanum_plugins/gazebo_mecanum_plugins_vel "f9df474704d7113e588f9ced0981881c")


