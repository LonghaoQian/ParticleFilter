;; Auto-generated. Do not edit!


(when (boundp 'ardrone_autonomy::navdata_altitude)
  (if (not (find-package "ARDRONE_AUTONOMY"))
    (make-package "ARDRONE_AUTONOMY"))
  (shadow 'navdata_altitude (find-package "ARDRONE_AUTONOMY")))
(unless (find-package "ARDRONE_AUTONOMY::NAVDATA_ALTITUDE")
  (make-package "ARDRONE_AUTONOMY::NAVDATA_ALTITUDE"))

(in-package "ROS")
;;//! \htmlinclude navdata_altitude.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ardrone_autonomy::navdata_altitude
  :super ros::object
  :slots (_header _drone_time _tag _size _altitude_vision _altitude_vz _altitude_ref _altitude_raw _obs_accZ _obs_alt _obs_x _obs_state _est_vb _est_state ))

(defmethod ardrone_autonomy::navdata_altitude
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:drone_time __drone_time) 0.0)
    ((:tag __tag) 0)
    ((:size __size) 0)
    ((:altitude_vision __altitude_vision) 0)
    ((:altitude_vz __altitude_vz) 0.0)
    ((:altitude_ref __altitude_ref) 0)
    ((:altitude_raw __altitude_raw) 0)
    ((:obs_accZ __obs_accZ) 0.0)
    ((:obs_alt __obs_alt) 0.0)
    ((:obs_x __obs_x) (instance ardrone_autonomy::vector31 :init))
    ((:obs_state __obs_state) 0)
    ((:est_vb __est_vb) (instance ardrone_autonomy::vector21 :init))
    ((:est_state __est_state) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _drone_time (float __drone_time))
   (setq _tag (round __tag))
   (setq _size (round __size))
   (setq _altitude_vision (round __altitude_vision))
   (setq _altitude_vz (float __altitude_vz))
   (setq _altitude_ref (round __altitude_ref))
   (setq _altitude_raw (round __altitude_raw))
   (setq _obs_accZ (float __obs_accZ))
   (setq _obs_alt (float __obs_alt))
   (setq _obs_x __obs_x)
   (setq _obs_state (round __obs_state))
   (setq _est_vb __est_vb)
   (setq _est_state (round __est_state))
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
  (:altitude_vision
   (&optional __altitude_vision)
   (if __altitude_vision (setq _altitude_vision __altitude_vision)) _altitude_vision)
  (:altitude_vz
   (&optional __altitude_vz)
   (if __altitude_vz (setq _altitude_vz __altitude_vz)) _altitude_vz)
  (:altitude_ref
   (&optional __altitude_ref)
   (if __altitude_ref (setq _altitude_ref __altitude_ref)) _altitude_ref)
  (:altitude_raw
   (&optional __altitude_raw)
   (if __altitude_raw (setq _altitude_raw __altitude_raw)) _altitude_raw)
  (:obs_accZ
   (&optional __obs_accZ)
   (if __obs_accZ (setq _obs_accZ __obs_accZ)) _obs_accZ)
  (:obs_alt
   (&optional __obs_alt)
   (if __obs_alt (setq _obs_alt __obs_alt)) _obs_alt)
  (:obs_x
   (&rest __obs_x)
   (if (keywordp (car __obs_x))
       (send* _obs_x __obs_x)
     (progn
       (if __obs_x (setq _obs_x (car __obs_x)))
       _obs_x)))
  (:obs_state
   (&optional __obs_state)
   (if __obs_state (setq _obs_state __obs_state)) _obs_state)
  (:est_vb
   (&rest __est_vb)
   (if (keywordp (car __est_vb))
       (send* _est_vb __est_vb)
     (progn
       (if __est_vb (setq _est_vb (car __est_vb)))
       _est_vb)))
  (:est_state
   (&optional __est_state)
   (if __est_state (setq _est_state __est_state)) _est_state)
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
    ;; int32 _altitude_vision
    4
    ;; float32 _altitude_vz
    4
    ;; int32 _altitude_ref
    4
    ;; int32 _altitude_raw
    4
    ;; float32 _obs_accZ
    4
    ;; float32 _obs_alt
    4
    ;; ardrone_autonomy/vector31 _obs_x
    (send _obs_x :serialization-length)
    ;; uint32 _obs_state
    4
    ;; ardrone_autonomy/vector21 _est_vb
    (send _est_vb :serialization-length)
    ;; uint32 _est_state
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
     ;; int32 _altitude_vision
       (write-long _altitude_vision s)
     ;; float32 _altitude_vz
       (sys::poke _altitude_vz (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32 _altitude_ref
       (write-long _altitude_ref s)
     ;; int32 _altitude_raw
       (write-long _altitude_raw s)
     ;; float32 _obs_accZ
       (sys::poke _obs_accZ (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _obs_alt
       (sys::poke _obs_alt (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; ardrone_autonomy/vector31 _obs_x
       (send _obs_x :serialize s)
     ;; uint32 _obs_state
       (write-long _obs_state s)
     ;; ardrone_autonomy/vector21 _est_vb
       (send _est_vb :serialize s)
     ;; uint32 _est_state
       (write-long _est_state s)
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
   ;; int32 _altitude_vision
     (setq _altitude_vision (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _altitude_vz
     (setq _altitude_vz (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32 _altitude_ref
     (setq _altitude_ref (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _altitude_raw
     (setq _altitude_raw (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _obs_accZ
     (setq _obs_accZ (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _obs_alt
     (setq _obs_alt (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; ardrone_autonomy/vector31 _obs_x
     (send _obs_x :deserialize buf ptr-) (incf ptr- (send _obs_x :serialization-length))
   ;; uint32 _obs_state
     (setq _obs_state (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; ardrone_autonomy/vector21 _est_vb
     (send _est_vb :deserialize buf ptr-) (incf ptr- (send _est_vb :serialization-length))
   ;; uint32 _est_state
     (setq _est_state (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get ardrone_autonomy::navdata_altitude :md5sum-) "f0fd1fd20697e6083c6bc3a308a260dc")
(setf (get ardrone_autonomy::navdata_altitude :datatype-) "ardrone_autonomy/navdata_altitude")
(setf (get ardrone_autonomy::navdata_altitude :definition-)
      "Header header
float64 drone_time
uint16 tag
uint16 size
int32 altitude_vision
float32 altitude_vz
int32 altitude_ref
int32 altitude_raw
float32 obs_accZ
float32 obs_alt
vector31 obs_x
uint32 obs_state
vector21 est_vb
uint32 est_state

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
================================================================================
MSG: ardrone_autonomy/vector21
float32 x
float32 y
")



(provide :ardrone_autonomy/navdata_altitude "f0fd1fd20697e6083c6bc3a308a260dc")


