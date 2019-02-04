;; Auto-generated. Do not edit!


(when (boundp 'ardrone_autonomy::navdata_trims)
  (if (not (find-package "ARDRONE_AUTONOMY"))
    (make-package "ARDRONE_AUTONOMY"))
  (shadow 'navdata_trims (find-package "ARDRONE_AUTONOMY")))
(unless (find-package "ARDRONE_AUTONOMY::NAVDATA_TRIMS")
  (make-package "ARDRONE_AUTONOMY::NAVDATA_TRIMS"))

(in-package "ROS")
;;//! \htmlinclude navdata_trims.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ardrone_autonomy::navdata_trims
  :super ros::object
  :slots (_header _drone_time _tag _size _angular_rates_trim_r _euler_angles_trim_theta _euler_angles_trim_phi ))

(defmethod ardrone_autonomy::navdata_trims
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:drone_time __drone_time) 0.0)
    ((:tag __tag) 0)
    ((:size __size) 0)
    ((:angular_rates_trim_r __angular_rates_trim_r) 0.0)
    ((:euler_angles_trim_theta __euler_angles_trim_theta) 0.0)
    ((:euler_angles_trim_phi __euler_angles_trim_phi) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _drone_time (float __drone_time))
   (setq _tag (round __tag))
   (setq _size (round __size))
   (setq _angular_rates_trim_r (float __angular_rates_trim_r))
   (setq _euler_angles_trim_theta (float __euler_angles_trim_theta))
   (setq _euler_angles_trim_phi (float __euler_angles_trim_phi))
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
  (:angular_rates_trim_r
   (&optional __angular_rates_trim_r)
   (if __angular_rates_trim_r (setq _angular_rates_trim_r __angular_rates_trim_r)) _angular_rates_trim_r)
  (:euler_angles_trim_theta
   (&optional __euler_angles_trim_theta)
   (if __euler_angles_trim_theta (setq _euler_angles_trim_theta __euler_angles_trim_theta)) _euler_angles_trim_theta)
  (:euler_angles_trim_phi
   (&optional __euler_angles_trim_phi)
   (if __euler_angles_trim_phi (setq _euler_angles_trim_phi __euler_angles_trim_phi)) _euler_angles_trim_phi)
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
    ;; float32 _angular_rates_trim_r
    4
    ;; float32 _euler_angles_trim_theta
    4
    ;; float32 _euler_angles_trim_phi
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
     ;; float32 _angular_rates_trim_r
       (sys::poke _angular_rates_trim_r (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _euler_angles_trim_theta
       (sys::poke _euler_angles_trim_theta (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _euler_angles_trim_phi
       (sys::poke _euler_angles_trim_phi (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
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
   ;; float32 _angular_rates_trim_r
     (setq _angular_rates_trim_r (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _euler_angles_trim_theta
     (setq _euler_angles_trim_theta (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _euler_angles_trim_phi
     (setq _euler_angles_trim_phi (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get ardrone_autonomy::navdata_trims :md5sum-) "71133941ec04f0892b7754f7e7191d05")
(setf (get ardrone_autonomy::navdata_trims :datatype-) "ardrone_autonomy/navdata_trims")
(setf (get ardrone_autonomy::navdata_trims :definition-)
      "Header header
float64 drone_time
uint16 tag
uint16 size
float32 angular_rates_trim_r
float32 euler_angles_trim_theta
float32 euler_angles_trim_phi

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



(provide :ardrone_autonomy/navdata_trims "71133941ec04f0892b7754f7e7191d05")


