;; Auto-generated. Do not edit!


(when (boundp 'ardrone_autonomy::navdata_demo)
  (if (not (find-package "ARDRONE_AUTONOMY"))
    (make-package "ARDRONE_AUTONOMY"))
  (shadow 'navdata_demo (find-package "ARDRONE_AUTONOMY")))
(unless (find-package "ARDRONE_AUTONOMY::NAVDATA_DEMO")
  (make-package "ARDRONE_AUTONOMY::NAVDATA_DEMO"))

(in-package "ROS")
;;//! \htmlinclude navdata_demo.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ardrone_autonomy::navdata_demo
  :super ros::object
  :slots (_header _drone_time _tag _size _ctrl_state _vbat_flying_percentage _theta _phi _psi _altitude _vx _vy _vz _num_frames _detection_camera_type ))

(defmethod ardrone_autonomy::navdata_demo
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:drone_time __drone_time) 0.0)
    ((:tag __tag) 0)
    ((:size __size) 0)
    ((:ctrl_state __ctrl_state) 0)
    ((:vbat_flying_percentage __vbat_flying_percentage) 0)
    ((:theta __theta) 0.0)
    ((:phi __phi) 0.0)
    ((:psi __psi) 0.0)
    ((:altitude __altitude) 0)
    ((:vx __vx) 0.0)
    ((:vy __vy) 0.0)
    ((:vz __vz) 0.0)
    ((:num_frames __num_frames) 0)
    ((:detection_camera_type __detection_camera_type) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _drone_time (float __drone_time))
   (setq _tag (round __tag))
   (setq _size (round __size))
   (setq _ctrl_state (round __ctrl_state))
   (setq _vbat_flying_percentage (round __vbat_flying_percentage))
   (setq _theta (float __theta))
   (setq _phi (float __phi))
   (setq _psi (float __psi))
   (setq _altitude (round __altitude))
   (setq _vx (float __vx))
   (setq _vy (float __vy))
   (setq _vz (float __vz))
   (setq _num_frames (round __num_frames))
   (setq _detection_camera_type (round __detection_camera_type))
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
  (:ctrl_state
   (&optional __ctrl_state)
   (if __ctrl_state (setq _ctrl_state __ctrl_state)) _ctrl_state)
  (:vbat_flying_percentage
   (&optional __vbat_flying_percentage)
   (if __vbat_flying_percentage (setq _vbat_flying_percentage __vbat_flying_percentage)) _vbat_flying_percentage)
  (:theta
   (&optional __theta)
   (if __theta (setq _theta __theta)) _theta)
  (:phi
   (&optional __phi)
   (if __phi (setq _phi __phi)) _phi)
  (:psi
   (&optional __psi)
   (if __psi (setq _psi __psi)) _psi)
  (:altitude
   (&optional __altitude)
   (if __altitude (setq _altitude __altitude)) _altitude)
  (:vx
   (&optional __vx)
   (if __vx (setq _vx __vx)) _vx)
  (:vy
   (&optional __vy)
   (if __vy (setq _vy __vy)) _vy)
  (:vz
   (&optional __vz)
   (if __vz (setq _vz __vz)) _vz)
  (:num_frames
   (&optional __num_frames)
   (if __num_frames (setq _num_frames __num_frames)) _num_frames)
  (:detection_camera_type
   (&optional __detection_camera_type)
   (if __detection_camera_type (setq _detection_camera_type __detection_camera_type)) _detection_camera_type)
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
    ;; uint32 _ctrl_state
    4
    ;; uint32 _vbat_flying_percentage
    4
    ;; float32 _theta
    4
    ;; float32 _phi
    4
    ;; float32 _psi
    4
    ;; int32 _altitude
    4
    ;; float32 _vx
    4
    ;; float32 _vy
    4
    ;; float32 _vz
    4
    ;; uint32 _num_frames
    4
    ;; uint32 _detection_camera_type
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
     ;; uint32 _ctrl_state
       (write-long _ctrl_state s)
     ;; uint32 _vbat_flying_percentage
       (write-long _vbat_flying_percentage s)
     ;; float32 _theta
       (sys::poke _theta (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _phi
       (sys::poke _phi (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _psi
       (sys::poke _psi (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32 _altitude
       (write-long _altitude s)
     ;; float32 _vx
       (sys::poke _vx (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _vy
       (sys::poke _vy (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _vz
       (sys::poke _vz (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint32 _num_frames
       (write-long _num_frames s)
     ;; uint32 _detection_camera_type
       (write-long _detection_camera_type s)
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
   ;; uint32 _ctrl_state
     (setq _ctrl_state (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _vbat_flying_percentage
     (setq _vbat_flying_percentage (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _theta
     (setq _theta (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _phi
     (setq _phi (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _psi
     (setq _psi (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32 _altitude
     (setq _altitude (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _vx
     (setq _vx (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _vy
     (setq _vy (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _vz
     (setq _vz (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint32 _num_frames
     (setq _num_frames (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _detection_camera_type
     (setq _detection_camera_type (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get ardrone_autonomy::navdata_demo :md5sum-) "5f569653293c884e2ea43f113057d0ea")
(setf (get ardrone_autonomy::navdata_demo :datatype-) "ardrone_autonomy/navdata_demo")
(setf (get ardrone_autonomy::navdata_demo :definition-)
      "Header header
float64 drone_time
uint16 tag
uint16 size
uint32 ctrl_state
uint32 vbat_flying_percentage
float32 theta
float32 phi
float32 psi
int32 altitude
float32 vx
float32 vy
float32 vz
uint32 num_frames
uint32 detection_camera_type

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



(provide :ardrone_autonomy/navdata_demo "5f569653293c884e2ea43f113057d0ea")


