;; Auto-generated. Do not edit!


(when (boundp 'ardrone_autonomy::navdata_pressure_raw)
  (if (not (find-package "ARDRONE_AUTONOMY"))
    (make-package "ARDRONE_AUTONOMY"))
  (shadow 'navdata_pressure_raw (find-package "ARDRONE_AUTONOMY")))
(unless (find-package "ARDRONE_AUTONOMY::NAVDATA_PRESSURE_RAW")
  (make-package "ARDRONE_AUTONOMY::NAVDATA_PRESSURE_RAW"))

(in-package "ROS")
;;//! \htmlinclude navdata_pressure_raw.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ardrone_autonomy::navdata_pressure_raw
  :super ros::object
  :slots (_header _drone_time _tag _size _up _ut _Temperature_meas _Pression_meas ))

(defmethod ardrone_autonomy::navdata_pressure_raw
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:drone_time __drone_time) 0.0)
    ((:tag __tag) 0)
    ((:size __size) 0)
    ((:up __up) 0)
    ((:ut __ut) 0)
    ((:Temperature_meas __Temperature_meas) 0)
    ((:Pression_meas __Pression_meas) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _drone_time (float __drone_time))
   (setq _tag (round __tag))
   (setq _size (round __size))
   (setq _up (round __up))
   (setq _ut (round __ut))
   (setq _Temperature_meas (round __Temperature_meas))
   (setq _Pression_meas (round __Pression_meas))
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
  (:up
   (&optional __up)
   (if __up (setq _up __up)) _up)
  (:ut
   (&optional __ut)
   (if __ut (setq _ut __ut)) _ut)
  (:Temperature_meas
   (&optional __Temperature_meas)
   (if __Temperature_meas (setq _Temperature_meas __Temperature_meas)) _Temperature_meas)
  (:Pression_meas
   (&optional __Pression_meas)
   (if __Pression_meas (setq _Pression_meas __Pression_meas)) _Pression_meas)
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
    ;; int32 _up
    4
    ;; int16 _ut
    2
    ;; int32 _Temperature_meas
    4
    ;; int32 _Pression_meas
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
     ;; int32 _up
       (write-long _up s)
     ;; int16 _ut
       (write-word _ut s)
     ;; int32 _Temperature_meas
       (write-long _Temperature_meas s)
     ;; int32 _Pression_meas
       (write-long _Pression_meas s)
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
   ;; int32 _up
     (setq _up (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int16 _ut
     (setq _ut (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int32 _Temperature_meas
     (setq _Temperature_meas (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _Pression_meas
     (setq _Pression_meas (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get ardrone_autonomy::navdata_pressure_raw :md5sum-) "bee798af5d1297cfcb39217d797b8d90")
(setf (get ardrone_autonomy::navdata_pressure_raw :datatype-) "ardrone_autonomy/navdata_pressure_raw")
(setf (get ardrone_autonomy::navdata_pressure_raw :definition-)
      "Header header
float64 drone_time
uint16 tag
uint16 size
int32 up
int16 ut
int32 Temperature_meas
int32 Pression_meas

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



(provide :ardrone_autonomy/navdata_pressure_raw "bee798af5d1297cfcb39217d797b8d90")


