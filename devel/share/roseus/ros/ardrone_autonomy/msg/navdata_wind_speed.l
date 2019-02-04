;; Auto-generated. Do not edit!


(when (boundp 'ardrone_autonomy::navdata_wind_speed)
  (if (not (find-package "ARDRONE_AUTONOMY"))
    (make-package "ARDRONE_AUTONOMY"))
  (shadow 'navdata_wind_speed (find-package "ARDRONE_AUTONOMY")))
(unless (find-package "ARDRONE_AUTONOMY::NAVDATA_WIND_SPEED")
  (make-package "ARDRONE_AUTONOMY::NAVDATA_WIND_SPEED"))

(in-package "ROS")
;;//! \htmlinclude navdata_wind_speed.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ardrone_autonomy::navdata_wind_speed
  :super ros::object
  :slots (_header _drone_time _tag _size _wind_speed _wind_angle _wind_compensation_theta _wind_compensation_phi _state_x1 _state_x2 _state_x3 _state_x4 _state_x5 _state_x6 _magneto_debug1 _magneto_debug2 _magneto_debug3 ))

(defmethod ardrone_autonomy::navdata_wind_speed
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:drone_time __drone_time) 0.0)
    ((:tag __tag) 0)
    ((:size __size) 0)
    ((:wind_speed __wind_speed) 0.0)
    ((:wind_angle __wind_angle) 0.0)
    ((:wind_compensation_theta __wind_compensation_theta) 0.0)
    ((:wind_compensation_phi __wind_compensation_phi) 0.0)
    ((:state_x1 __state_x1) 0.0)
    ((:state_x2 __state_x2) 0.0)
    ((:state_x3 __state_x3) 0.0)
    ((:state_x4 __state_x4) 0.0)
    ((:state_x5 __state_x5) 0.0)
    ((:state_x6 __state_x6) 0.0)
    ((:magneto_debug1 __magneto_debug1) 0.0)
    ((:magneto_debug2 __magneto_debug2) 0.0)
    ((:magneto_debug3 __magneto_debug3) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _drone_time (float __drone_time))
   (setq _tag (round __tag))
   (setq _size (round __size))
   (setq _wind_speed (float __wind_speed))
   (setq _wind_angle (float __wind_angle))
   (setq _wind_compensation_theta (float __wind_compensation_theta))
   (setq _wind_compensation_phi (float __wind_compensation_phi))
   (setq _state_x1 (float __state_x1))
   (setq _state_x2 (float __state_x2))
   (setq _state_x3 (float __state_x3))
   (setq _state_x4 (float __state_x4))
   (setq _state_x5 (float __state_x5))
   (setq _state_x6 (float __state_x6))
   (setq _magneto_debug1 (float __magneto_debug1))
   (setq _magneto_debug2 (float __magneto_debug2))
   (setq _magneto_debug3 (float __magneto_debug3))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:drone_time
   (&optional __drone_time)
   (if __drone_time (setq _drone_time __drone_time)) _drone_time)
  (:tag
   (&optional __tag)
   (if __tag (setq _tag __tag)) _tag)
  (:size
   (&optional __size)
   (if __size (setq _size __size)) _size)
  (:wind_speed
   (&optional __wind_speed)
   (if __wind_speed (setq _wind_speed __wind_speed)) _wind_speed)
  (:wind_angle
   (&optional __wind_angle)
   (if __wind_angle (setq _wind_angle __wind_angle)) _wind_angle)
  (:wind_compensation_theta
   (&optional __wind_compensation_theta)
   (if __wind_compensation_theta (setq _wind_compensation_theta __wind_compensation_theta)) _wind_compensation_theta)
  (:wind_compensation_phi
   (&optional __wind_compensation_phi)
   (if __wind_compensation_phi (setq _wind_compensation_phi __wind_compensation_phi)) _wind_compensation_phi)
  (:state_x1
   (&optional __state_x1)
   (if __state_x1 (setq _state_x1 __state_x1)) _state_x1)
  (:state_x2
   (&optional __state_x2)
   (if __state_x2 (setq _state_x2 __state_x2)) _state_x2)
  (:state_x3
   (&optional __state_x3)
   (if __state_x3 (setq _state_x3 __state_x3)) _state_x3)
  (:state_x4
   (&optional __state_x4)
   (if __state_x4 (setq _state_x4 __state_x4)) _state_x4)
  (:state_x5
   (&optional __state_x5)
   (if __state_x5 (setq _state_x5 __state_x5)) _state_x5)
  (:state_x6
   (&optional __state_x6)
   (if __state_x6 (setq _state_x6 __state_x6)) _state_x6)
  (:magneto_debug1
   (&optional __magneto_debug1)
   (if __magneto_debug1 (setq _magneto_debug1 __magneto_debug1)) _magneto_debug1)
  (:magneto_debug2
   (&optional __magneto_debug2)
   (if __magneto_debug2 (setq _magneto_debug2 __magneto_debug2)) _magneto_debug2)
  (:magneto_debug3
   (&optional __magneto_debug3)
   (if __magneto_debug3 (setq _magneto_debug3 __magneto_debug3)) _magneto_debug3)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _drone_time
    8
    ;; uint16 _tag
    2
    ;; uint16 _size
    2
    ;; float32 _wind_speed
    4
    ;; float32 _wind_angle
    4
    ;; float32 _wind_compensation_theta
    4
    ;; float32 _wind_compensation_phi
    4
    ;; float32 _state_x1
    4
    ;; float32 _state_x2
    4
    ;; float32 _state_x3
    4
    ;; float32 _state_x4
    4
    ;; float32 _state_x5
    4
    ;; float32 _state_x6
    4
    ;; float32 _magneto_debug1
    4
    ;; float32 _magneto_debug2
    4
    ;; float32 _magneto_debug3
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _drone_time
       (sys::poke _drone_time (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; uint16 _tag
       (write-word _tag s)
     ;; uint16 _size
       (write-word _size s)
     ;; float32 _wind_speed
       (sys::poke _wind_speed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _wind_angle
       (sys::poke _wind_angle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _wind_compensation_theta
       (sys::poke _wind_compensation_theta (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _wind_compensation_phi
       (sys::poke _wind_compensation_phi (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _state_x1
       (sys::poke _state_x1 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _state_x2
       (sys::poke _state_x2 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _state_x3
       (sys::poke _state_x3 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _state_x4
       (sys::poke _state_x4 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _state_x5
       (sys::poke _state_x5 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _state_x6
       (sys::poke _state_x6 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _magneto_debug1
       (sys::poke _magneto_debug1 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _magneto_debug2
       (sys::poke _magneto_debug2 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _magneto_debug3
       (sys::poke _magneto_debug3 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _drone_time
     (setq _drone_time (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; uint16 _tag
     (setq _tag (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _size
     (setq _size (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float32 _wind_speed
     (setq _wind_speed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _wind_angle
     (setq _wind_angle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _wind_compensation_theta
     (setq _wind_compensation_theta (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _wind_compensation_phi
     (setq _wind_compensation_phi (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _state_x1
     (setq _state_x1 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _state_x2
     (setq _state_x2 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _state_x3
     (setq _state_x3 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _state_x4
     (setq _state_x4 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _state_x5
     (setq _state_x5 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _state_x6
     (setq _state_x6 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _magneto_debug1
     (setq _magneto_debug1 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _magneto_debug2
     (setq _magneto_debug2 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _magneto_debug3
     (setq _magneto_debug3 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get ardrone_autonomy::navdata_wind_speed :md5sum-) "2cc5c1e9675c330dd38261e958a136d3")
(setf (get ardrone_autonomy::navdata_wind_speed :datatype-) "ardrone_autonomy/navdata_wind_speed")
(setf (get ardrone_autonomy::navdata_wind_speed :definition-)
      "Header header
float64 drone_time
uint16 tag
uint16 size
float32 wind_speed
float32 wind_angle
float32 wind_compensation_theta
float32 wind_compensation_phi
float32 state_x1
float32 state_x2
float32 state_x3
float32 state_x4
float32 state_x5
float32 state_x6
float32 magneto_debug1
float32 magneto_debug2
float32 magneto_debug3

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

")



(provide :ardrone_autonomy/navdata_wind_speed "2cc5c1e9675c330dd38261e958a136d3")


