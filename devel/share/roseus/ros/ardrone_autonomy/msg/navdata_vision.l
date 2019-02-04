;; Auto-generated. Do not edit!


(when (boundp 'ardrone_autonomy::navdata_vision)
  (if (not (find-package "ARDRONE_AUTONOMY"))
    (make-package "ARDRONE_AUTONOMY"))
  (shadow 'navdata_vision (find-package "ARDRONE_AUTONOMY")))
(unless (find-package "ARDRONE_AUTONOMY::NAVDATA_VISION")
  (make-package "ARDRONE_AUTONOMY::NAVDATA_VISION"))

(in-package "ROS")
;;//! \htmlinclude navdata_vision.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ardrone_autonomy::navdata_vision
  :super ros::object
  :slots (_header _drone_time _tag _size _vision_state _vision_misc _vision_phi_trim _vision_phi_ref_prop _vision_theta_trim _vision_theta_ref_prop _new_raw_picture _theta_capture _phi_capture _psi_capture _altitude_capture _time_capture _body_v _delta_phi _delta_theta _delta_psi _gold_defined _gold_reset _gold_x _gold_y ))

(defmethod ardrone_autonomy::navdata_vision
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:drone_time __drone_time) 0.0)
    ((:tag __tag) 0)
    ((:size __size) 0)
    ((:vision_state __vision_state) 0)
    ((:vision_misc __vision_misc) 0)
    ((:vision_phi_trim __vision_phi_trim) 0.0)
    ((:vision_phi_ref_prop __vision_phi_ref_prop) 0.0)
    ((:vision_theta_trim __vision_theta_trim) 0.0)
    ((:vision_theta_ref_prop __vision_theta_ref_prop) 0.0)
    ((:new_raw_picture __new_raw_picture) 0)
    ((:theta_capture __theta_capture) 0.0)
    ((:phi_capture __phi_capture) 0.0)
    ((:psi_capture __psi_capture) 0.0)
    ((:altitude_capture __altitude_capture) 0)
    ((:time_capture __time_capture) 0)
    ((:body_v __body_v) (instance ardrone_autonomy::vector31 :init))
    ((:delta_phi __delta_phi) 0.0)
    ((:delta_theta __delta_theta) 0.0)
    ((:delta_psi __delta_psi) 0.0)
    ((:gold_defined __gold_defined) 0)
    ((:gold_reset __gold_reset) 0)
    ((:gold_x __gold_x) 0.0)
    ((:gold_y __gold_y) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _drone_time (float __drone_time))
   (setq _tag (round __tag))
   (setq _size (round __size))
   (setq _vision_state (round __vision_state))
   (setq _vision_misc (round __vision_misc))
   (setq _vision_phi_trim (float __vision_phi_trim))
   (setq _vision_phi_ref_prop (float __vision_phi_ref_prop))
   (setq _vision_theta_trim (float __vision_theta_trim))
   (setq _vision_theta_ref_prop (float __vision_theta_ref_prop))
   (setq _new_raw_picture (round __new_raw_picture))
   (setq _theta_capture (float __theta_capture))
   (setq _phi_capture (float __phi_capture))
   (setq _psi_capture (float __psi_capture))
   (setq _altitude_capture (round __altitude_capture))
   (setq _time_capture (round __time_capture))
   (setq _body_v __body_v)
   (setq _delta_phi (float __delta_phi))
   (setq _delta_theta (float __delta_theta))
   (setq _delta_psi (float __delta_psi))
   (setq _gold_defined (round __gold_defined))
   (setq _gold_reset (round __gold_reset))
   (setq _gold_x (float __gold_x))
   (setq _gold_y (float __gold_y))
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
  (:vision_state
   (&optional __vision_state)
   (if __vision_state (setq _vision_state __vision_state)) _vision_state)
  (:vision_misc
   (&optional __vision_misc)
   (if __vision_misc (setq _vision_misc __vision_misc)) _vision_misc)
  (:vision_phi_trim
   (&optional __vision_phi_trim)
   (if __vision_phi_trim (setq _vision_phi_trim __vision_phi_trim)) _vision_phi_trim)
  (:vision_phi_ref_prop
   (&optional __vision_phi_ref_prop)
   (if __vision_phi_ref_prop (setq _vision_phi_ref_prop __vision_phi_ref_prop)) _vision_phi_ref_prop)
  (:vision_theta_trim
   (&optional __vision_theta_trim)
   (if __vision_theta_trim (setq _vision_theta_trim __vision_theta_trim)) _vision_theta_trim)
  (:vision_theta_ref_prop
   (&optional __vision_theta_ref_prop)
   (if __vision_theta_ref_prop (setq _vision_theta_ref_prop __vision_theta_ref_prop)) _vision_theta_ref_prop)
  (:new_raw_picture
   (&optional __new_raw_picture)
   (if __new_raw_picture (setq _new_raw_picture __new_raw_picture)) _new_raw_picture)
  (:theta_capture
   (&optional __theta_capture)
   (if __theta_capture (setq _theta_capture __theta_capture)) _theta_capture)
  (:phi_capture
   (&optional __phi_capture)
   (if __phi_capture (setq _phi_capture __phi_capture)) _phi_capture)
  (:psi_capture
   (&optional __psi_capture)
   (if __psi_capture (setq _psi_capture __psi_capture)) _psi_capture)
  (:altitude_capture
   (&optional __altitude_capture)
   (if __altitude_capture (setq _altitude_capture __altitude_capture)) _altitude_capture)
  (:time_capture
   (&optional __time_capture)
   (if __time_capture (setq _time_capture __time_capture)) _time_capture)
  (:body_v
   (&rest __body_v)
   (if (keywordp (car __body_v))
       (send* _body_v __body_v)
     (progn
       (if __body_v (setq _body_v (car __body_v)))
       _body_v)))
  (:delta_phi
   (&optional __delta_phi)
   (if __delta_phi (setq _delta_phi __delta_phi)) _delta_phi)
  (:delta_theta
   (&optional __delta_theta)
   (if __delta_theta (setq _delta_theta __delta_theta)) _delta_theta)
  (:delta_psi
   (&optional __delta_psi)
   (if __delta_psi (setq _delta_psi __delta_psi)) _delta_psi)
  (:gold_defined
   (&optional __gold_defined)
   (if __gold_defined (setq _gold_defined __gold_defined)) _gold_defined)
  (:gold_reset
   (&optional __gold_reset)
   (if __gold_reset (setq _gold_reset __gold_reset)) _gold_reset)
  (:gold_x
   (&optional __gold_x)
   (if __gold_x (setq _gold_x __gold_x)) _gold_x)
  (:gold_y
   (&optional __gold_y)
   (if __gold_y (setq _gold_y __gold_y)) _gold_y)
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
    ;; uint32 _vision_state
    4
    ;; int32 _vision_misc
    4
    ;; float32 _vision_phi_trim
    4
    ;; float32 _vision_phi_ref_prop
    4
    ;; float32 _vision_theta_trim
    4
    ;; float32 _vision_theta_ref_prop
    4
    ;; int32 _new_raw_picture
    4
    ;; float32 _theta_capture
    4
    ;; float32 _phi_capture
    4
    ;; float32 _psi_capture
    4
    ;; int32 _altitude_capture
    4
    ;; uint32 _time_capture
    4
    ;; ardrone_autonomy/vector31 _body_v
    (send _body_v :serialization-length)
    ;; float32 _delta_phi
    4
    ;; float32 _delta_theta
    4
    ;; float32 _delta_psi
    4
    ;; uint32 _gold_defined
    4
    ;; uint32 _gold_reset
    4
    ;; float32 _gold_x
    4
    ;; float32 _gold_y
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
     ;; uint32 _vision_state
       (write-long _vision_state s)
     ;; int32 _vision_misc
       (write-long _vision_misc s)
     ;; float32 _vision_phi_trim
       (sys::poke _vision_phi_trim (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _vision_phi_ref_prop
       (sys::poke _vision_phi_ref_prop (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _vision_theta_trim
       (sys::poke _vision_theta_trim (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _vision_theta_ref_prop
       (sys::poke _vision_theta_ref_prop (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32 _new_raw_picture
       (write-long _new_raw_picture s)
     ;; float32 _theta_capture
       (sys::poke _theta_capture (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _phi_capture
       (sys::poke _phi_capture (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _psi_capture
       (sys::poke _psi_capture (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32 _altitude_capture
       (write-long _altitude_capture s)
     ;; uint32 _time_capture
       (write-long _time_capture s)
     ;; ardrone_autonomy/vector31 _body_v
       (send _body_v :serialize s)
     ;; float32 _delta_phi
       (sys::poke _delta_phi (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _delta_theta
       (sys::poke _delta_theta (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _delta_psi
       (sys::poke _delta_psi (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint32 _gold_defined
       (write-long _gold_defined s)
     ;; uint32 _gold_reset
       (write-long _gold_reset s)
     ;; float32 _gold_x
       (sys::poke _gold_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _gold_y
       (sys::poke _gold_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
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
   ;; uint32 _vision_state
     (setq _vision_state (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _vision_misc
     (setq _vision_misc (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _vision_phi_trim
     (setq _vision_phi_trim (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _vision_phi_ref_prop
     (setq _vision_phi_ref_prop (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _vision_theta_trim
     (setq _vision_theta_trim (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _vision_theta_ref_prop
     (setq _vision_theta_ref_prop (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32 _new_raw_picture
     (setq _new_raw_picture (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _theta_capture
     (setq _theta_capture (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _phi_capture
     (setq _phi_capture (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _psi_capture
     (setq _psi_capture (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32 _altitude_capture
     (setq _altitude_capture (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _time_capture
     (setq _time_capture (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; ardrone_autonomy/vector31 _body_v
     (send _body_v :deserialize buf ptr-) (incf ptr- (send _body_v :serialization-length))
   ;; float32 _delta_phi
     (setq _delta_phi (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _delta_theta
     (setq _delta_theta (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _delta_psi
     (setq _delta_psi (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint32 _gold_defined
     (setq _gold_defined (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _gold_reset
     (setq _gold_reset (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _gold_x
     (setq _gold_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _gold_y
     (setq _gold_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get ardrone_autonomy::navdata_vision :md5sum-) "55dd9a6c13917332d6d39025ed1318ab")
(setf (get ardrone_autonomy::navdata_vision :datatype-) "ardrone_autonomy/navdata_vision")
(setf (get ardrone_autonomy::navdata_vision :definition-)
      "Header header
float64 drone_time
uint16 tag
uint16 size
uint32 vision_state
int32 vision_misc
float32 vision_phi_trim
float32 vision_phi_ref_prop
float32 vision_theta_trim
float32 vision_theta_ref_prop
int32 new_raw_picture
float32 theta_capture
float32 phi_capture
float32 psi_capture
int32 altitude_capture
uint32 time_capture
vector31 body_v
float32 delta_phi
float32 delta_theta
float32 delta_psi
uint32 gold_defined
uint32 gold_reset
float32 gold_x
float32 gold_y

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

================================================================================
MSG: ardrone_autonomy/vector31
float32 x
float32 y
float32 z
")



(provide :ardrone_autonomy/navdata_vision "55dd9a6c13917332d6d39025ed1318ab")


