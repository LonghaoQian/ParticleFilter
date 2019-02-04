;; Auto-generated. Do not edit!


(when (boundp 'ardrone_autonomy::navdata_vision_raw)
  (if (not (find-package "ARDRONE_AUTONOMY"))
    (make-package "ARDRONE_AUTONOMY"))
  (shadow 'navdata_vision_raw (find-package "ARDRONE_AUTONOMY")))
(unless (find-package "ARDRONE_AUTONOMY::NAVDATA_VISION_RAW")
  (make-package "ARDRONE_AUTONOMY::NAVDATA_VISION_RAW"))

(in-package "ROS")
;;//! \htmlinclude navdata_vision_raw.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ardrone_autonomy::navdata_vision_raw
  :super ros::object
  :slots (_header _drone_time _tag _size _vision_tx_raw _vision_ty_raw _vision_tz_raw ))

(defmethod ardrone_autonomy::navdata_vision_raw
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:drone_time __drone_time) 0.0)
    ((:tag __tag) 0)
    ((:size __size) 0)
    ((:vision_tx_raw __vision_tx_raw) 0.0)
    ((:vision_ty_raw __vision_ty_raw) 0.0)
    ((:vision_tz_raw __vision_tz_raw) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _drone_time (float __drone_time))
   (setq _tag (round __tag))
   (setq _size (round __size))
   (setq _vision_tx_raw (float __vision_tx_raw))
   (setq _vision_ty_raw (float __vision_ty_raw))
   (setq _vision_tz_raw (float __vision_tz_raw))
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
  (:vision_tx_raw
   (&optional __vision_tx_raw)
   (if __vision_tx_raw (setq _vision_tx_raw __vision_tx_raw)) _vision_tx_raw)
  (:vision_ty_raw
   (&optional __vision_ty_raw)
   (if __vision_ty_raw (setq _vision_ty_raw __vision_ty_raw)) _vision_ty_raw)
  (:vision_tz_raw
   (&optional __vision_tz_raw)
   (if __vision_tz_raw (setq _vision_tz_raw __vision_tz_raw)) _vision_tz_raw)
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
    ;; float32 _vision_tx_raw
    4
    ;; float32 _vision_ty_raw
    4
    ;; float32 _vision_tz_raw
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
     ;; float32 _vision_tx_raw
       (sys::poke _vision_tx_raw (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _vision_ty_raw
       (sys::poke _vision_ty_raw (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _vision_tz_raw
       (sys::poke _vision_tz_raw (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
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
   ;; float32 _vision_tx_raw
     (setq _vision_tx_raw (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _vision_ty_raw
     (setq _vision_ty_raw (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _vision_tz_raw
     (setq _vision_tz_raw (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get ardrone_autonomy::navdata_vision_raw :md5sum-) "a656de33995320e1ee5a84370378836e")
(setf (get ardrone_autonomy::navdata_vision_raw :datatype-) "ardrone_autonomy/navdata_vision_raw")
(setf (get ardrone_autonomy::navdata_vision_raw :definition-)
      "Header header
float64 drone_time
uint16 tag
uint16 size
float32 vision_tx_raw
float32 vision_ty_raw
float32 vision_tz_raw

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



(provide :ardrone_autonomy/navdata_vision_raw "a656de33995320e1ee5a84370378836e")


