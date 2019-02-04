;; Auto-generated. Do not edit!


(when (boundp 'ardrone_autonomy::navdata_vision_perf)
  (if (not (find-package "ARDRONE_AUTONOMY"))
    (make-package "ARDRONE_AUTONOMY"))
  (shadow 'navdata_vision_perf (find-package "ARDRONE_AUTONOMY")))
(unless (find-package "ARDRONE_AUTONOMY::NAVDATA_VISION_PERF")
  (make-package "ARDRONE_AUTONOMY::NAVDATA_VISION_PERF"))

(in-package "ROS")
;;//! \htmlinclude navdata_vision_perf.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ardrone_autonomy::navdata_vision_perf
  :super ros::object
  :slots (_header _drone_time _tag _size _time_corners _time_compute _time_tracking _time_trans _time_update _time_custom ))

(defmethod ardrone_autonomy::navdata_vision_perf
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:drone_time __drone_time) 0.0)
    ((:tag __tag) 0)
    ((:size __size) 0)
    ((:time_corners __time_corners) 0.0)
    ((:time_compute __time_compute) 0.0)
    ((:time_tracking __time_tracking) 0.0)
    ((:time_trans __time_trans) 0.0)
    ((:time_update __time_update) 0.0)
    ((:time_custom __time_custom) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _drone_time (float __drone_time))
   (setq _tag (round __tag))
   (setq _size (round __size))
   (setq _time_corners (float __time_corners))
   (setq _time_compute (float __time_compute))
   (setq _time_tracking (float __time_tracking))
   (setq _time_trans (float __time_trans))
   (setq _time_update (float __time_update))
   (setq _time_custom __time_custom)
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
  (:time_corners
   (&optional __time_corners)
   (if __time_corners (setq _time_corners __time_corners)) _time_corners)
  (:time_compute
   (&optional __time_compute)
   (if __time_compute (setq _time_compute __time_compute)) _time_compute)
  (:time_tracking
   (&optional __time_tracking)
   (if __time_tracking (setq _time_tracking __time_tracking)) _time_tracking)
  (:time_trans
   (&optional __time_trans)
   (if __time_trans (setq _time_trans __time_trans)) _time_trans)
  (:time_update
   (&optional __time_update)
   (if __time_update (setq _time_update __time_update)) _time_update)
  (:time_custom
   (&optional __time_custom)
   (if __time_custom (setq _time_custom __time_custom)) _time_custom)
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
    ;; float32 _time_corners
    4
    ;; float32 _time_compute
    4
    ;; float32 _time_tracking
    4
    ;; float32 _time_trans
    4
    ;; float32 _time_update
    4
    ;; float32[] _time_custom
    (* 4    (length _time_custom)) 4
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
     ;; float32 _time_corners
       (sys::poke _time_corners (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _time_compute
       (sys::poke _time_compute (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _time_tracking
       (sys::poke _time_tracking (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _time_trans
       (sys::poke _time_trans (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _time_update
       (sys::poke _time_update (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32[] _time_custom
     (write-long (length _time_custom) s)
     (dotimes (i (length _time_custom))
       (sys::poke (elt _time_custom i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
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
   ;; float32 _time_corners
     (setq _time_corners (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _time_compute
     (setq _time_compute (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _time_tracking
     (setq _time_tracking (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _time_trans
     (setq _time_trans (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _time_update
     (setq _time_update (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32[] _time_custom
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _time_custom (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _time_custom i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get ardrone_autonomy::navdata_vision_perf :md5sum-) "5ed8267a2e2980a430a3439af8e5c9f7")
(setf (get ardrone_autonomy::navdata_vision_perf :datatype-) "ardrone_autonomy/navdata_vision_perf")
(setf (get ardrone_autonomy::navdata_vision_perf :definition-)
      "Header header
float64 drone_time
uint16 tag
uint16 size
float32 time_corners
float32 time_compute
float32 time_tracking
float32 time_trans
float32 time_update
float32[] time_custom

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



(provide :ardrone_autonomy/navdata_vision_perf "5ed8267a2e2980a430a3439af8e5c9f7")


