;; Auto-generated. Do not edit!


(when (boundp 'ardrone_autonomy::navdata_references)
  (if (not (find-package "ARDRONE_AUTONOMY"))
    (make-package "ARDRONE_AUTONOMY"))
  (shadow 'navdata_references (find-package "ARDRONE_AUTONOMY")))
(unless (find-package "ARDRONE_AUTONOMY::NAVDATA_REFERENCES")
  (make-package "ARDRONE_AUTONOMY::NAVDATA_REFERENCES"))

(in-package "ROS")
;;//! \htmlinclude navdata_references.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ardrone_autonomy::navdata_references
  :super ros::object
  :slots (_header _drone_time _tag _size _ref_theta _ref_phi _ref_theta_I _ref_phi_I _ref_pitch _ref_roll _ref_yaw _ref_psi _vx_ref _vy_ref _theta_mod _phi_mod _k_v_x _k_v_y _k_mode _ui_time _ui_theta _ui_phi _ui_psi _ui_psi_accuracy _ui_seq ))

(defmethod ardrone_autonomy::navdata_references
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:drone_time __drone_time) 0.0)
    ((:tag __tag) 0)
    ((:size __size) 0)
    ((:ref_theta __ref_theta) 0)
    ((:ref_phi __ref_phi) 0)
    ((:ref_theta_I __ref_theta_I) 0)
    ((:ref_phi_I __ref_phi_I) 0)
    ((:ref_pitch __ref_pitch) 0)
    ((:ref_roll __ref_roll) 0)
    ((:ref_yaw __ref_yaw) 0)
    ((:ref_psi __ref_psi) 0)
    ((:vx_ref __vx_ref) 0.0)
    ((:vy_ref __vy_ref) 0.0)
    ((:theta_mod __theta_mod) 0.0)
    ((:phi_mod __phi_mod) 0.0)
    ((:k_v_x __k_v_x) 0.0)
    ((:k_v_y __k_v_y) 0.0)
    ((:k_mode __k_mode) 0)
    ((:ui_time __ui_time) 0.0)
    ((:ui_theta __ui_theta) 0.0)
    ((:ui_phi __ui_phi) 0.0)
    ((:ui_psi __ui_psi) 0.0)
    ((:ui_psi_accuracy __ui_psi_accuracy) 0.0)
    ((:ui_seq __ui_seq) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _drone_time (float __drone_time))
   (setq _tag (round __tag))
   (setq _size (round __size))
   (setq _ref_theta (round __ref_theta))
   (setq _ref_phi (round __ref_phi))
   (setq _ref_theta_I (round __ref_theta_I))
   (setq _ref_phi_I (round __ref_phi_I))
   (setq _ref_pitch (round __ref_pitch))
   (setq _ref_roll (round __ref_roll))
   (setq _ref_yaw (round __ref_yaw))
   (setq _ref_psi (round __ref_psi))
   (setq _vx_ref (float __vx_ref))
   (setq _vy_ref (float __vy_ref))
   (setq _theta_mod (float __theta_mod))
   (setq _phi_mod (float __phi_mod))
   (setq _k_v_x (float __k_v_x))
   (setq _k_v_y (float __k_v_y))
   (setq _k_mode (round __k_mode))
   (setq _ui_time (float __ui_time))
   (setq _ui_theta (float __ui_theta))
   (setq _ui_phi (float __ui_phi))
   (setq _ui_psi (float __ui_psi))
   (setq _ui_psi_accuracy (float __ui_psi_accuracy))
   (setq _ui_seq (round __ui_seq))
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
  (:ref_theta
   (&optional __ref_theta)
   (if __ref_theta (setq _ref_theta __ref_theta)) _ref_theta)
  (:ref_phi
   (&optional __ref_phi)
   (if __ref_phi (setq _ref_phi __ref_phi)) _ref_phi)
  (:ref_theta_I
   (&optional __ref_theta_I)
   (if __ref_theta_I (setq _ref_theta_I __ref_theta_I)) _ref_theta_I)
  (:ref_phi_I
   (&optional __ref_phi_I)
   (if __ref_phi_I (setq _ref_phi_I __ref_phi_I)) _ref_phi_I)
  (:ref_pitch
   (&optional __ref_pitch)
   (if __ref_pitch (setq _ref_pitch __ref_pitch)) _ref_pitch)
  (:ref_roll
   (&optional __ref_roll)
   (if __ref_roll (setq _ref_roll __ref_roll)) _ref_roll)
  (:ref_yaw
   (&optional __ref_yaw)
   (if __ref_yaw (setq _ref_yaw __ref_yaw)) _ref_yaw)
  (:ref_psi
   (&optional __ref_psi)
   (if __ref_psi (setq _ref_psi __ref_psi)) _ref_psi)
  (:vx_ref
   (&optional __vx_ref)
   (if __vx_ref (setq _vx_ref __vx_ref)) _vx_ref)
  (:vy_ref
   (&optional __vy_ref)
   (if __vy_ref (setq _vy_ref __vy_ref)) _vy_ref)
  (:theta_mod
   (&optional __theta_mod)
   (if __theta_mod (setq _theta_mod __theta_mod)) _theta_mod)
  (:phi_mod
   (&optional __phi_mod)
   (if __phi_mod (setq _phi_mod __phi_mod)) _phi_mod)
  (:k_v_x
   (&optional __k_v_x)
   (if __k_v_x (setq _k_v_x __k_v_x)) _k_v_x)
  (:k_v_y
   (&optional __k_v_y)
   (if __k_v_y (setq _k_v_y __k_v_y)) _k_v_y)
  (:k_mode
   (&optional __k_mode)
   (if __k_mode (setq _k_mode __k_mode)) _k_mode)
  (:ui_time
   (&optional __ui_time)
   (if __ui_time (setq _ui_time __ui_time)) _ui_time)
  (:ui_theta
   (&optional __ui_theta)
   (if __ui_theta (setq _ui_theta __ui_theta)) _ui_theta)
  (:ui_phi
   (&optional __ui_phi)
   (if __ui_phi (setq _ui_phi __ui_phi)) _ui_phi)
  (:ui_psi
   (&optional __ui_psi)
   (if __ui_psi (setq _ui_psi __ui_psi)) _ui_psi)
  (:ui_psi_accuracy
   (&optional __ui_psi_accuracy)
   (if __ui_psi_accuracy (setq _ui_psi_accuracy __ui_psi_accuracy)) _ui_psi_accuracy)
  (:ui_seq
   (&optional __ui_seq)
   (if __ui_seq (setq _ui_seq __ui_seq)) _ui_seq)
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
    ;; int32 _ref_theta
    4
    ;; int32 _ref_phi
    4
    ;; int32 _ref_theta_I
    4
    ;; int32 _ref_phi_I
    4
    ;; int32 _ref_pitch
    4
    ;; int32 _ref_roll
    4
    ;; int32 _ref_yaw
    4
    ;; int32 _ref_psi
    4
    ;; float32 _vx_ref
    4
    ;; float32 _vy_ref
    4
    ;; float32 _theta_mod
    4
    ;; float32 _phi_mod
    4
    ;; float32 _k_v_x
    4
    ;; float32 _k_v_y
    4
    ;; uint32 _k_mode
    4
    ;; float32 _ui_time
    4
    ;; float32 _ui_theta
    4
    ;; float32 _ui_phi
    4
    ;; float32 _ui_psi
    4
    ;; float32 _ui_psi_accuracy
    4
    ;; int32 _ui_seq
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
     ;; int32 _ref_theta
       (write-long _ref_theta s)
     ;; int32 _ref_phi
       (write-long _ref_phi s)
     ;; int32 _ref_theta_I
       (write-long _ref_theta_I s)
     ;; int32 _ref_phi_I
       (write-long _ref_phi_I s)
     ;; int32 _ref_pitch
       (write-long _ref_pitch s)
     ;; int32 _ref_roll
       (write-long _ref_roll s)
     ;; int32 _ref_yaw
       (write-long _ref_yaw s)
     ;; int32 _ref_psi
       (write-long _ref_psi s)
     ;; float32 _vx_ref
       (sys::poke _vx_ref (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _vy_ref
       (sys::poke _vy_ref (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _theta_mod
       (sys::poke _theta_mod (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _phi_mod
       (sys::poke _phi_mod (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _k_v_x
       (sys::poke _k_v_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _k_v_y
       (sys::poke _k_v_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint32 _k_mode
       (write-long _k_mode s)
     ;; float32 _ui_time
       (sys::poke _ui_time (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _ui_theta
       (sys::poke _ui_theta (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _ui_phi
       (sys::poke _ui_phi (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _ui_psi
       (sys::poke _ui_psi (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _ui_psi_accuracy
       (sys::poke _ui_psi_accuracy (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32 _ui_seq
       (write-long _ui_seq s)
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
   ;; int32 _ref_theta
     (setq _ref_theta (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _ref_phi
     (setq _ref_phi (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _ref_theta_I
     (setq _ref_theta_I (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _ref_phi_I
     (setq _ref_phi_I (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _ref_pitch
     (setq _ref_pitch (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _ref_roll
     (setq _ref_roll (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _ref_yaw
     (setq _ref_yaw (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _ref_psi
     (setq _ref_psi (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _vx_ref
     (setq _vx_ref (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _vy_ref
     (setq _vy_ref (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _theta_mod
     (setq _theta_mod (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _phi_mod
     (setq _phi_mod (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _k_v_x
     (setq _k_v_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _k_v_y
     (setq _k_v_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint32 _k_mode
     (setq _k_mode (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _ui_time
     (setq _ui_time (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _ui_theta
     (setq _ui_theta (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _ui_phi
     (setq _ui_phi (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _ui_psi
     (setq _ui_psi (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _ui_psi_accuracy
     (setq _ui_psi_accuracy (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32 _ui_seq
     (setq _ui_seq (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get ardrone_autonomy::navdata_references :md5sum-) "263b844b053f4a098c75c1c26a452911")
(setf (get ardrone_autonomy::navdata_references :datatype-) "ardrone_autonomy/navdata_references")
(setf (get ardrone_autonomy::navdata_references :definition-)
      "Header header
float64 drone_time
uint16 tag
uint16 size
int32 ref_theta
int32 ref_phi
int32 ref_theta_I
int32 ref_phi_I
int32 ref_pitch
int32 ref_roll
int32 ref_yaw
int32 ref_psi
float32 vx_ref
float32 vy_ref
float32 theta_mod
float32 phi_mod
float32 k_v_x
float32 k_v_y
uint32 k_mode
float32 ui_time
float32 ui_theta
float32 ui_phi
float32 ui_psi
float32 ui_psi_accuracy
int32 ui_seq

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



(provide :ardrone_autonomy/navdata_references "263b844b053f4a098c75c1c26a452911")


