;; Auto-generated. Do not edit!


(when (boundp 'ardrone_autonomy::navdata_pwm)
  (if (not (find-package "ARDRONE_AUTONOMY"))
    (make-package "ARDRONE_AUTONOMY"))
  (shadow 'navdata_pwm (find-package "ARDRONE_AUTONOMY")))
(unless (find-package "ARDRONE_AUTONOMY::NAVDATA_PWM")
  (make-package "ARDRONE_AUTONOMY::NAVDATA_PWM"))

(in-package "ROS")
;;//! \htmlinclude navdata_pwm.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ardrone_autonomy::navdata_pwm
  :super ros::object
  :slots (_header _drone_time _tag _size _motor1 _motor2 _motor3 _motor4 _sat_motor1 _sat_motor2 _sat_motor3 _sat_motor4 _gaz_feed_forward _gaz_altitude _altitude_integral _vz_ref _u_pitch _u_roll _u_yaw _yaw_u_I _u_pitch_planif _u_roll_planif _u_yaw_planif _u_gaz_planif _current_motor1 _current_motor2 _current_motor3 _current_motor4 _altitude_der ))

(defmethod ardrone_autonomy::navdata_pwm
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:drone_time __drone_time) 0.0)
    ((:tag __tag) 0)
    ((:size __size) 0)
    ((:motor1 __motor1) 0)
    ((:motor2 __motor2) 0)
    ((:motor3 __motor3) 0)
    ((:motor4 __motor4) 0)
    ((:sat_motor1 __sat_motor1) 0)
    ((:sat_motor2 __sat_motor2) 0)
    ((:sat_motor3 __sat_motor3) 0)
    ((:sat_motor4 __sat_motor4) 0)
    ((:gaz_feed_forward __gaz_feed_forward) 0.0)
    ((:gaz_altitude __gaz_altitude) 0.0)
    ((:altitude_integral __altitude_integral) 0.0)
    ((:vz_ref __vz_ref) 0.0)
    ((:u_pitch __u_pitch) 0)
    ((:u_roll __u_roll) 0)
    ((:u_yaw __u_yaw) 0)
    ((:yaw_u_I __yaw_u_I) 0.0)
    ((:u_pitch_planif __u_pitch_planif) 0)
    ((:u_roll_planif __u_roll_planif) 0)
    ((:u_yaw_planif __u_yaw_planif) 0)
    ((:u_gaz_planif __u_gaz_planif) 0.0)
    ((:current_motor1 __current_motor1) 0)
    ((:current_motor2 __current_motor2) 0)
    ((:current_motor3 __current_motor3) 0)
    ((:current_motor4 __current_motor4) 0)
    ((:altitude_der __altitude_der) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _drone_time (float __drone_time))
   (setq _tag (round __tag))
   (setq _size (round __size))
   (setq _motor1 (round __motor1))
   (setq _motor2 (round __motor2))
   (setq _motor3 (round __motor3))
   (setq _motor4 (round __motor4))
   (setq _sat_motor1 (round __sat_motor1))
   (setq _sat_motor2 (round __sat_motor2))
   (setq _sat_motor3 (round __sat_motor3))
   (setq _sat_motor4 (round __sat_motor4))
   (setq _gaz_feed_forward (float __gaz_feed_forward))
   (setq _gaz_altitude (float __gaz_altitude))
   (setq _altitude_integral (float __altitude_integral))
   (setq _vz_ref (float __vz_ref))
   (setq _u_pitch (round __u_pitch))
   (setq _u_roll (round __u_roll))
   (setq _u_yaw (round __u_yaw))
   (setq _yaw_u_I (float __yaw_u_I))
   (setq _u_pitch_planif (round __u_pitch_planif))
   (setq _u_roll_planif (round __u_roll_planif))
   (setq _u_yaw_planif (round __u_yaw_planif))
   (setq _u_gaz_planif (float __u_gaz_planif))
   (setq _current_motor1 (round __current_motor1))
   (setq _current_motor2 (round __current_motor2))
   (setq _current_motor3 (round __current_motor3))
   (setq _current_motor4 (round __current_motor4))
   (setq _altitude_der (float __altitude_der))
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
  (:motor1
   (&optional __motor1)
   (if __motor1 (setq _motor1 __motor1)) _motor1)
  (:motor2
   (&optional __motor2)
   (if __motor2 (setq _motor2 __motor2)) _motor2)
  (:motor3
   (&optional __motor3)
   (if __motor3 (setq _motor3 __motor3)) _motor3)
  (:motor4
   (&optional __motor4)
   (if __motor4 (setq _motor4 __motor4)) _motor4)
  (:sat_motor1
   (&optional __sat_motor1)
   (if __sat_motor1 (setq _sat_motor1 __sat_motor1)) _sat_motor1)
  (:sat_motor2
   (&optional __sat_motor2)
   (if __sat_motor2 (setq _sat_motor2 __sat_motor2)) _sat_motor2)
  (:sat_motor3
   (&optional __sat_motor3)
   (if __sat_motor3 (setq _sat_motor3 __sat_motor3)) _sat_motor3)
  (:sat_motor4
   (&optional __sat_motor4)
   (if __sat_motor4 (setq _sat_motor4 __sat_motor4)) _sat_motor4)
  (:gaz_feed_forward
   (&optional __gaz_feed_forward)
   (if __gaz_feed_forward (setq _gaz_feed_forward __gaz_feed_forward)) _gaz_feed_forward)
  (:gaz_altitude
   (&optional __gaz_altitude)
   (if __gaz_altitude (setq _gaz_altitude __gaz_altitude)) _gaz_altitude)
  (:altitude_integral
   (&optional __altitude_integral)
   (if __altitude_integral (setq _altitude_integral __altitude_integral)) _altitude_integral)
  (:vz_ref
   (&optional __vz_ref)
   (if __vz_ref (setq _vz_ref __vz_ref)) _vz_ref)
  (:u_pitch
   (&optional __u_pitch)
   (if __u_pitch (setq _u_pitch __u_pitch)) _u_pitch)
  (:u_roll
   (&optional __u_roll)
   (if __u_roll (setq _u_roll __u_roll)) _u_roll)
  (:u_yaw
   (&optional __u_yaw)
   (if __u_yaw (setq _u_yaw __u_yaw)) _u_yaw)
  (:yaw_u_I
   (&optional __yaw_u_I)
   (if __yaw_u_I (setq _yaw_u_I __yaw_u_I)) _yaw_u_I)
  (:u_pitch_planif
   (&optional __u_pitch_planif)
   (if __u_pitch_planif (setq _u_pitch_planif __u_pitch_planif)) _u_pitch_planif)
  (:u_roll_planif
   (&optional __u_roll_planif)
   (if __u_roll_planif (setq _u_roll_planif __u_roll_planif)) _u_roll_planif)
  (:u_yaw_planif
   (&optional __u_yaw_planif)
   (if __u_yaw_planif (setq _u_yaw_planif __u_yaw_planif)) _u_yaw_planif)
  (:u_gaz_planif
   (&optional __u_gaz_planif)
   (if __u_gaz_planif (setq _u_gaz_planif __u_gaz_planif)) _u_gaz_planif)
  (:current_motor1
   (&optional __current_motor1)
   (if __current_motor1 (setq _current_motor1 __current_motor1)) _current_motor1)
  (:current_motor2
   (&optional __current_motor2)
   (if __current_motor2 (setq _current_motor2 __current_motor2)) _current_motor2)
  (:current_motor3
   (&optional __current_motor3)
   (if __current_motor3 (setq _current_motor3 __current_motor3)) _current_motor3)
  (:current_motor4
   (&optional __current_motor4)
   (if __current_motor4 (setq _current_motor4 __current_motor4)) _current_motor4)
  (:altitude_der
   (&optional __altitude_der)
   (if __altitude_der (setq _altitude_der __altitude_der)) _altitude_der)
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
    ;; uint8 _motor1
    1
    ;; uint8 _motor2
    1
    ;; uint8 _motor3
    1
    ;; uint8 _motor4
    1
    ;; uint8 _sat_motor1
    1
    ;; uint8 _sat_motor2
    1
    ;; uint8 _sat_motor3
    1
    ;; uint8 _sat_motor4
    1
    ;; float32 _gaz_feed_forward
    4
    ;; float32 _gaz_altitude
    4
    ;; float32 _altitude_integral
    4
    ;; float32 _vz_ref
    4
    ;; int32 _u_pitch
    4
    ;; int32 _u_roll
    4
    ;; int32 _u_yaw
    4
    ;; float32 _yaw_u_I
    4
    ;; int32 _u_pitch_planif
    4
    ;; int32 _u_roll_planif
    4
    ;; int32 _u_yaw_planif
    4
    ;; float32 _u_gaz_planif
    4
    ;; uint16 _current_motor1
    2
    ;; uint16 _current_motor2
    2
    ;; uint16 _current_motor3
    2
    ;; uint16 _current_motor4
    2
    ;; float32 _altitude_der
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
     ;; uint8 _motor1
       (write-byte _motor1 s)
     ;; uint8 _motor2
       (write-byte _motor2 s)
     ;; uint8 _motor3
       (write-byte _motor3 s)
     ;; uint8 _motor4
       (write-byte _motor4 s)
     ;; uint8 _sat_motor1
       (write-byte _sat_motor1 s)
     ;; uint8 _sat_motor2
       (write-byte _sat_motor2 s)
     ;; uint8 _sat_motor3
       (write-byte _sat_motor3 s)
     ;; uint8 _sat_motor4
       (write-byte _sat_motor4 s)
     ;; float32 _gaz_feed_forward
       (sys::poke _gaz_feed_forward (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _gaz_altitude
       (sys::poke _gaz_altitude (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _altitude_integral
       (sys::poke _altitude_integral (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _vz_ref
       (sys::poke _vz_ref (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32 _u_pitch
       (write-long _u_pitch s)
     ;; int32 _u_roll
       (write-long _u_roll s)
     ;; int32 _u_yaw
       (write-long _u_yaw s)
     ;; float32 _yaw_u_I
       (sys::poke _yaw_u_I (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32 _u_pitch_planif
       (write-long _u_pitch_planif s)
     ;; int32 _u_roll_planif
       (write-long _u_roll_planif s)
     ;; int32 _u_yaw_planif
       (write-long _u_yaw_planif s)
     ;; float32 _u_gaz_planif
       (sys::poke _u_gaz_planif (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint16 _current_motor1
       (write-word _current_motor1 s)
     ;; uint16 _current_motor2
       (write-word _current_motor2 s)
     ;; uint16 _current_motor3
       (write-word _current_motor3 s)
     ;; uint16 _current_motor4
       (write-word _current_motor4 s)
     ;; float32 _altitude_der
       (sys::poke _altitude_der (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
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
   ;; uint8 _motor1
     (setq _motor1 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _motor2
     (setq _motor2 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _motor3
     (setq _motor3 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _motor4
     (setq _motor4 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _sat_motor1
     (setq _sat_motor1 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _sat_motor2
     (setq _sat_motor2 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _sat_motor3
     (setq _sat_motor3 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _sat_motor4
     (setq _sat_motor4 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _gaz_feed_forward
     (setq _gaz_feed_forward (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _gaz_altitude
     (setq _gaz_altitude (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _altitude_integral
     (setq _altitude_integral (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _vz_ref
     (setq _vz_ref (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32 _u_pitch
     (setq _u_pitch (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _u_roll
     (setq _u_roll (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _u_yaw
     (setq _u_yaw (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _yaw_u_I
     (setq _yaw_u_I (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32 _u_pitch_planif
     (setq _u_pitch_planif (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _u_roll_planif
     (setq _u_roll_planif (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _u_yaw_planif
     (setq _u_yaw_planif (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _u_gaz_planif
     (setq _u_gaz_planif (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint16 _current_motor1
     (setq _current_motor1 (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _current_motor2
     (setq _current_motor2 (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _current_motor3
     (setq _current_motor3 (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _current_motor4
     (setq _current_motor4 (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float32 _altitude_der
     (setq _altitude_der (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get ardrone_autonomy::navdata_pwm :md5sum-) "6369e637a869960c68d7de5c2e10d6b1")
(setf (get ardrone_autonomy::navdata_pwm :datatype-) "ardrone_autonomy/navdata_pwm")
(setf (get ardrone_autonomy::navdata_pwm :definition-)
      "Header header
float64 drone_time
uint16 tag
uint16 size
uint8 motor1
uint8 motor2
uint8 motor3
uint8 motor4
uint8 sat_motor1
uint8 sat_motor2
uint8 sat_motor3
uint8 sat_motor4
float32 gaz_feed_forward
float32 gaz_altitude
float32 altitude_integral
float32 vz_ref
int32 u_pitch
int32 u_roll
int32 u_yaw
float32 yaw_u_I
int32 u_pitch_planif
int32 u_roll_planif
int32 u_yaw_planif
float32 u_gaz_planif
uint16 current_motor1
uint16 current_motor2
uint16 current_motor3
uint16 current_motor4
float32 altitude_der

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



(provide :ardrone_autonomy/navdata_pwm "6369e637a869960c68d7de5c2e10d6b1")


