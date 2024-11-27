;; Auto-generated. Do not edit!


(when (boundp 'gazebo_mecanum_plugins::gazebo_mecanum_plugins_pid)
  (if (not (find-package "GAZEBO_MECANUM_PLUGINS"))
    (make-package "GAZEBO_MECANUM_PLUGINS"))
  (shadow 'gazebo_mecanum_plugins_pid (find-package "GAZEBO_MECANUM_PLUGINS")))
(unless (find-package "GAZEBO_MECANUM_PLUGINS::GAZEBO_MECANUM_PLUGINS_PID")
  (make-package "GAZEBO_MECANUM_PLUGINS::GAZEBO_MECANUM_PLUGINS_PID"))

(in-package "ROS")
;;//! \htmlinclude gazebo_mecanum_plugins_pid.msg.html


(defclass gazebo_mecanum_plugins::gazebo_mecanum_plugins_pid
  :super ros::object
  :slots (_LR_P _LR_I _LR_D _LF_P _LF_I _LF_D _RF_P _RF_I _RF_D _RR_P _RR_I _RR_D ))

(defmethod gazebo_mecanum_plugins::gazebo_mecanum_plugins_pid
  (:init
   (&key
    ((:LR_P __LR_P) 0.0)
    ((:LR_I __LR_I) 0.0)
    ((:LR_D __LR_D) 0.0)
    ((:LF_P __LF_P) 0.0)
    ((:LF_I __LF_I) 0.0)
    ((:LF_D __LF_D) 0.0)
    ((:RF_P __RF_P) 0.0)
    ((:RF_I __RF_I) 0.0)
    ((:RF_D __RF_D) 0.0)
    ((:RR_P __RR_P) 0.0)
    ((:RR_I __RR_I) 0.0)
    ((:RR_D __RR_D) 0.0)
    )
   (send-super :init)
   (setq _LR_P (float __LR_P))
   (setq _LR_I (float __LR_I))
   (setq _LR_D (float __LR_D))
   (setq _LF_P (float __LF_P))
   (setq _LF_I (float __LF_I))
   (setq _LF_D (float __LF_D))
   (setq _RF_P (float __RF_P))
   (setq _RF_I (float __RF_I))
   (setq _RF_D (float __RF_D))
   (setq _RR_P (float __RR_P))
   (setq _RR_I (float __RR_I))
   (setq _RR_D (float __RR_D))
   self)
  (:LR_P
   (&optional __LR_P)
   (if __LR_P (setq _LR_P __LR_P)) _LR_P)
  (:LR_I
   (&optional __LR_I)
   (if __LR_I (setq _LR_I __LR_I)) _LR_I)
  (:LR_D
   (&optional __LR_D)
   (if __LR_D (setq _LR_D __LR_D)) _LR_D)
  (:LF_P
   (&optional __LF_P)
   (if __LF_P (setq _LF_P __LF_P)) _LF_P)
  (:LF_I
   (&optional __LF_I)
   (if __LF_I (setq _LF_I __LF_I)) _LF_I)
  (:LF_D
   (&optional __LF_D)
   (if __LF_D (setq _LF_D __LF_D)) _LF_D)
  (:RF_P
   (&optional __RF_P)
   (if __RF_P (setq _RF_P __RF_P)) _RF_P)
  (:RF_I
   (&optional __RF_I)
   (if __RF_I (setq _RF_I __RF_I)) _RF_I)
  (:RF_D
   (&optional __RF_D)
   (if __RF_D (setq _RF_D __RF_D)) _RF_D)
  (:RR_P
   (&optional __RR_P)
   (if __RR_P (setq _RR_P __RR_P)) _RR_P)
  (:RR_I
   (&optional __RR_I)
   (if __RR_I (setq _RR_I __RR_I)) _RR_I)
  (:RR_D
   (&optional __RR_D)
   (if __RR_D (setq _RR_D __RR_D)) _RR_D)
  (:serialization-length
   ()
   (+
    ;; float64 _LR_P
    8
    ;; float64 _LR_I
    8
    ;; float64 _LR_D
    8
    ;; float64 _LF_P
    8
    ;; float64 _LF_I
    8
    ;; float64 _LF_D
    8
    ;; float64 _RF_P
    8
    ;; float64 _RF_I
    8
    ;; float64 _RF_D
    8
    ;; float64 _RR_P
    8
    ;; float64 _RR_I
    8
    ;; float64 _RR_D
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _LR_P
       (sys::poke _LR_P (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _LR_I
       (sys::poke _LR_I (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _LR_D
       (sys::poke _LR_D (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _LF_P
       (sys::poke _LF_P (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _LF_I
       (sys::poke _LF_I (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _LF_D
       (sys::poke _LF_D (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _RF_P
       (sys::poke _RF_P (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _RF_I
       (sys::poke _RF_I (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _RF_D
       (sys::poke _RF_D (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _RR_P
       (sys::poke _RR_P (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _RR_I
       (sys::poke _RR_I (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _RR_D
       (sys::poke _RR_D (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _LR_P
     (setq _LR_P (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _LR_I
     (setq _LR_I (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _LR_D
     (setq _LR_D (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _LF_P
     (setq _LF_P (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _LF_I
     (setq _LF_I (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _LF_D
     (setq _LF_D (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _RF_P
     (setq _RF_P (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _RF_I
     (setq _RF_I (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _RF_D
     (setq _RF_D (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _RR_P
     (setq _RR_P (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _RR_I
     (setq _RR_I (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _RR_D
     (setq _RR_D (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get gazebo_mecanum_plugins::gazebo_mecanum_plugins_pid :md5sum-) "9eb89efea79b06cdde55828e8174754b")
(setf (get gazebo_mecanum_plugins::gazebo_mecanum_plugins_pid :datatype-) "gazebo_mecanum_plugins/gazebo_mecanum_plugins_pid")
(setf (get gazebo_mecanum_plugins::gazebo_mecanum_plugins_pid :definition-)
      "float64 LR_P
float64 LR_I
float64 LR_D
float64 LF_P
float64 LF_I
float64 LF_D
float64 RF_P
float64 RF_I
float64 RF_D
float64 RR_P
float64 RR_I
float64 RR_D
")



(provide :gazebo_mecanum_plugins/gazebo_mecanum_plugins_pid "9eb89efea79b06cdde55828e8174754b")


