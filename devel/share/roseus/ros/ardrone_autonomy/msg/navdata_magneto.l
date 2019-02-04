;; Auto-generated. Do not edit!


(when (boundp 'ardrone_autonomy::navdata_magneto)
  (if (not (find-package "ARDRONE_AUTONOMY"))
    (make-package "ARDRONE_AUTONOMY"))
  (shadow 'navdata_magneto (find-package "ARDRONE_AUTONOMY")))
(unless (find-package "ARDRONE_AUTONOMY::NAVDATA_MAGNETO")
  (make-package "ARDRONE_AUTONOMY::NAVDATA_MAGNETO"))

(in-package "ROS")
;;//! \htmlinclude navdata_magneto.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ardrone_autonomy::navdata_magneto
  :super ros::object
  :slots (_header _drone_time _tag _size _mx _my _mz _magneto_raw _magneto_rectified _magneto_offset _heading_unwrapped _heading_gyro_unwrapped _heading_fusion_unwrapped _magneto_calibration_ok _magneto_state _magneto_radius _error_mean _error_var ))

(defmethod ardrone_autonomy::navdata_magneto
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:drone_time __drone_time) 0.0)
    ((:tag __tag) 0)
    ((:size __size) 0)
    ((:mx __mx) 0)
    ((:my __my) 0)
    ((:mz __mz) 0)
    ((:magneto_raw __magneto_raw) (instance ardrone_autonomy::vector31 :init))
    ((:magneto_rectified __magneto_rectified) (instance ardrone_autonomy::vector31 :init))
    ((:magneto_offset __magneto_offset) (instance ardrone_autonomy::vector31 :init))
    ((:heading_unwrapped __heading_unwrapped) 0.0)
    ((:heading_gyro_unwrapped __heading_gyro_unwrapped) 0.0)
    ((:heading_fusion_unwrapped __heading_fusion_unwrapped) 0.0)
    ((:magneto_calibration_ok __magneto_calibration_ok) 0)
    ((:magneto_state __magneto_state) 0)
    ((:magneto_radius __magneto_radius) 0.0)
    ((:error_mean __error_mean) 0.0)
    ((:error_var __error_var) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _drone_time (float __drone_time))
   (setq _tag (round __tag))
   (setq _size (round __size))
   (setq _mx (round __mx))
   (setq _my (round __my))
   (setq _mz (round __mz))
   (setq _magneto_raw __magneto_raw)
   (setq _magneto_rectified __magneto_rectified)
   (setq _magneto_offset __magneto_offset)
   (setq _heading_unwrapped (float __heading_unwrapped))
   (setq _heading_gyro_unwrapped (float __heading_gyro_unwrapped))
   (setq _heading_fusion_unwrapped (float __heading_fusion_unwrapped))
   (setq _magneto_calibration_ok (round __magneto_calibration_ok))
   (setq _magneto_state (round __magneto_state))
   (setq _magneto_radius (float __magneto_radius))
   (setq _error_mean (float __error_mean))
   (setq _error_var (float __error_var))
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
  (:mx
   (&optional __mx)
   (if __mx (setq _mx __mx)) _mx)
  (:my
   (&optional __my)
   (if __my (setq _my __my)) _my)
  (:mz
   (&optional __mz)
   (if __mz (setq _mz __mz)) _mz)
  (:magneto_raw
   (&rest __magneto_raw)
   (if (keywordp (car __magneto_raw))
       (send* _magneto_raw __magneto_raw)
     (progn
       (if __magneto_raw (setq _magneto_raw (car __magneto_raw)))
       _magneto_raw)))
  (:magneto_rectified
   (&rest __magneto_rectified)
   (if (keywordp (car __magneto_rectified))
       (send* _magneto_rectified __magneto_rectified)
     (progn
       (if __magneto_rectified (setq _magneto_rectified (car __magneto_rectified)))
       _magneto_rectified)))
  (:magneto_offset
   (&rest __magneto_offset)
   (if (keywordp (car __magneto_offset))
       (send* _magneto_offset __magneto_offset)
     (progn
       (if __magneto_offset (setq _magneto_offset (car __magneto_offset)))
       _magneto_offset)))
  (:heading_unwrapped
   (&optional __heading_unwrapped)
   (if __heading_unwrapped (setq _heading_unwrapped __heading_unwrapped)) _heading_unwrapped)
  (:heading_gyro_unwrapped
   (&optional __heading_gyro_unwrapped)
   (if __heading_gyro_unwrapped (setq _heading_gyro_unwrapped __heading_gyro_unwrapped)) _heading_gyro_unwrapped)
  (:heading_fusion_unwrapped
   (&optional __heading_fusion_unwrapped)
   (if __heading_fusion_unwrapped (setq _heading_fusion_unwrapped __heading_fusion_unwrapped)) _heading_fusion_unwrapped)
  (:magneto_calibration_ok
   (&optional __magneto_calibration_ok)
   (if __magneto_calibration_ok (setq _magneto_calibration_ok __magneto_calibration_ok)) _magneto_calibration_ok)
  (:magneto_state
   (&optional __magneto_state)
   (if __magneto_state (setq _magneto_state __magneto_state)) _magneto_state)
  (:magneto_radius
   (&optional __magneto_radius)
   (if __magneto_radius (setq _magneto_radius __magneto_radius)) _magneto_radius)
  (:error_mean
   (&optional __error_mean)
   (if __error_mean (setq _error_mean __error_mean)) _error_mean)
  (:error_var
   (&optional __error_var)
   (if __error_var (setq _error_var __error_var)) _error_var)
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
    ;; int16 _mx
    2
    ;; int16 _my
    2
    ;; int16 _mz
    2
    ;; ardrone_autonomy/vector31 _magneto_raw
    (send _magneto_raw :serialization-length)
    ;; ardrone_autonomy/vector31 _magneto_rectified
    (send _magneto_rectified :serialization-length)
    ;; ardrone_autonomy/vector31 _magneto_offset
    (send _magneto_offset :serialization-length)
    ;; float32 _heading_unwrapped
    4
    ;; float32 _heading_gyro_unwrapped
    4
    ;; float32 _heading_fusion_unwrapped
    4
    ;; uint8 _magneto_calibration_ok
    1
    ;; uint32 _magneto_state
    4
    ;; float32 _magneto_radius
    4
    ;; float32 _error_mean
    4
    ;; float32 _error_var
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
     ;; int16 _mx
       (write-word _mx s)
     ;; int16 _my
       (write-word _my s)
     ;; int16 _mz
       (write-word _mz s)
     ;; ardrone_autonomy/vector31 _magneto_raw
       (send _magneto_raw :serialize s)
     ;; ardrone_autonomy/vector31 _magneto_rectified
       (send _magneto_rectified :serialize s)
     ;; ardrone_autonomy/vector31 _magneto_offset
       (send _magneto_offset :serialize s)
     ;; float32 _heading_unwrapped
       (sys::poke _heading_unwrapped (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _heading_gyro_unwrapped
       (sys::poke _heading_gyro_unwrapped (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _heading_fusion_unwrapped
       (sys::poke _heading_fusion_unwrapped (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint8 _magneto_calibration_ok
       (write-byte _magneto_calibration_ok s)
     ;; uint32 _magneto_state
       (write-long _magneto_state s)
     ;; float32 _magneto_radius
       (sys::poke _magneto_radius (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _error_mean
       (sys::poke _error_mean (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _error_var
       (sys::poke _error_var (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
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
   ;; int16 _mx
     (setq _mx (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _my
     (setq _my (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _mz
     (setq _mz (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; ardrone_autonomy/vector31 _magneto_raw
     (send _magneto_raw :deserialize buf ptr-) (incf ptr- (send _magneto_raw :serialization-length))
   ;; ardrone_autonomy/vector31 _magneto_rectified
     (send _magneto_rectified :deserialize buf ptr-) (incf ptr- (send _magneto_rectified :serialization-length))
   ;; ardrone_autonomy/vector31 _magneto_offset
     (send _magneto_offset :deserialize buf ptr-) (incf ptr- (send _magneto_offset :serialization-length))
   ;; float32 _heading_unwrapped
     (setq _heading_unwrapped (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _heading_gyro_unwrapped
     (setq _heading_gyro_unwrapped (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _heading_fusion_unwrapped
     (setq _heading_fusion_unwrapped (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint8 _magneto_calibration_ok
     (setq _magneto_calibration_ok (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint32 _magneto_state
     (setq _magneto_state (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _magneto_radius
     (setq _magneto_radius (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _error_mean
     (setq _error_mean (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _error_var
     (setq _error_var (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get ardrone_autonomy::navdata_magneto :md5sum-) "c256b1c1d1ff0cb12a13c36720b3e224")
(setf (get ardrone_autonomy::navdata_magneto :datatype-) "ardrone_autonomy/navdata_magneto")
(setf (get ardrone_autonomy::navdata_magneto :definition-)
      "Header header
float64 drone_time
uint16 tag
uint16 size
int16 mx
int16 my
int16 mz
vector31 magneto_raw
vector31 magneto_rectified
vector31 magneto_offset
float32 heading_unwrapped
float32 heading_gyro_unwrapped
float32 heading_fusion_unwrapped
uint8 magneto_calibration_ok
uint32 magneto_state
float32 magneto_radius
float32 error_mean
float32 error_var

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



(provide :ardrone_autonomy/navdata_magneto "c256b1c1d1ff0cb12a13c36720b3e224")


