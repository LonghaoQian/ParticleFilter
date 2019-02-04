;; Auto-generated. Do not edit!


(when (boundp 'ardrone_autonomy::navdata_watchdog)
  (if (not (find-package "ARDRONE_AUTONOMY"))
    (make-package "ARDRONE_AUTONOMY"))
  (shadow 'navdata_watchdog (find-package "ARDRONE_AUTONOMY")))
(unless (find-package "ARDRONE_AUTONOMY::NAVDATA_WATCHDOG")
  (make-package "ARDRONE_AUTONOMY::NAVDATA_WATCHDOG"))

(in-package "ROS")
;;//! \htmlinclude navdata_watchdog.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ardrone_autonomy::navdata_watchdog
  :super ros::object
  :slots (_header _drone_time _tag _size ))

(defmethod ardrone_autonomy::navdata_watchdog
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:drone_time __drone_time) 0.0)
    ((:tag __tag) 0)
    ((:size __size) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _drone_time (float __drone_time))
   (setq _tag (round __tag))
   (setq _size (round __size))
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
   ;;
   self)
  )

(setf (get ardrone_autonomy::navdata_watchdog :md5sum-) "9e64269f1f5e463ea4f48c395e917507")
(setf (get ardrone_autonomy::navdata_watchdog :datatype-) "ardrone_autonomy/navdata_watchdog")
(setf (get ardrone_autonomy::navdata_watchdog :definition-)
      "Header header
float64 drone_time
uint16 tag
uint16 size

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



(provide :ardrone_autonomy/navdata_watchdog "9e64269f1f5e463ea4f48c395e917507")


