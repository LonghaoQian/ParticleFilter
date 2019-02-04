;; Auto-generated. Do not edit!


(when (boundp 'ardrone_autonomy::navdata_euler_angles)
  (if (not (find-package "ARDRONE_AUTONOMY"))
    (make-package "ARDRONE_AUTONOMY"))
  (shadow 'navdata_euler_angles (find-package "ARDRONE_AUTONOMY")))
(unless (find-package "ARDRONE_AUTONOMY::NAVDATA_EULER_ANGLES")
  (make-package "ARDRONE_AUTONOMY::NAVDATA_EULER_ANGLES"))

(in-package "ROS")
;;//! \htmlinclude navdata_euler_angles.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ardrone_autonomy::navdata_euler_angles
  :super ros::object
  :slots (_header _drone_time _tag _size _theta_a _phi_a ))

(defmethod ardrone_autonomy::navdata_euler_angles
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:drone_time __drone_time) 0.0)
    ((:tag __tag) 0)
    ((:size __size) 0)
    ((:theta_a __theta_a) 0.0)
    ((:phi_a __phi_a) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _drone_time (float __drone_time))
   (setq _tag (round __tag))
   (setq _size (round __size))
   (setq _theta_a (float __theta_a))
   (setq _phi_a (float __phi_a))
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
  (:theta_a
   (&optional __theta_a)
   (if __theta_a (setq _theta_a __theta_a)) _theta_a)
  (:phi_a
   (&optional __phi_a)
   (if __phi_a (setq _phi_a __phi_a)) _phi_a)
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
    ;; float32 _theta_a
    4
    ;; float32 _phi_a
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
     ;; float32 _theta_a
       (sys::poke _theta_a (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _phi_a
       (sys::poke _phi_a (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
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
   ;; float32 _theta_a
     (setq _theta_a (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _phi_a
     (setq _phi_a (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get ardrone_autonomy::navdata_euler_angles :md5sum-) "ce675f5dfa1b9c65f4844f2311d307aa")
(setf (get ardrone_autonomy::navdata_euler_angles :datatype-) "ardrone_autonomy/navdata_euler_angles")
(setf (get ardrone_autonomy::navdata_euler_angles :definition-)
      "Header header
float64 drone_time
uint16 tag
uint16 size
float32 theta_a
float32 phi_a

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



(provide :ardrone_autonomy/navdata_euler_angles "ce675f5dfa1b9c65f4844f2311d307aa")


