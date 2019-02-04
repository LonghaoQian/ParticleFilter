;; Auto-generated. Do not edit!


(when (boundp 'ardrone_autonomy::navdata_rc_references)
  (if (not (find-package "ARDRONE_AUTONOMY"))
    (make-package "ARDRONE_AUTONOMY"))
  (shadow 'navdata_rc_references (find-package "ARDRONE_AUTONOMY")))
(unless (find-package "ARDRONE_AUTONOMY::NAVDATA_RC_REFERENCES")
  (make-package "ARDRONE_AUTONOMY::NAVDATA_RC_REFERENCES"))

(in-package "ROS")
;;//! \htmlinclude navdata_rc_references.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ardrone_autonomy::navdata_rc_references
  :super ros::object
  :slots (_header _drone_time _tag _size _rc_ref_pitch _rc_ref_roll _rc_ref_yaw _rc_ref_gaz _rc_ref_ag ))

(defmethod ardrone_autonomy::navdata_rc_references
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:drone_time __drone_time) 0.0)
    ((:tag __tag) 0)
    ((:size __size) 0)
    ((:rc_ref_pitch __rc_ref_pitch) 0)
    ((:rc_ref_roll __rc_ref_roll) 0)
    ((:rc_ref_yaw __rc_ref_yaw) 0)
    ((:rc_ref_gaz __rc_ref_gaz) 0)
    ((:rc_ref_ag __rc_ref_ag) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _drone_time (float __drone_time))
   (setq _tag (round __tag))
   (setq _size (round __size))
   (setq _rc_ref_pitch (round __rc_ref_pitch))
   (setq _rc_ref_roll (round __rc_ref_roll))
   (setq _rc_ref_yaw (round __rc_ref_yaw))
   (setq _rc_ref_gaz (round __rc_ref_gaz))
   (setq _rc_ref_ag (round __rc_ref_ag))
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
  (:rc_ref_pitch
   (&optional __rc_ref_pitch)
   (if __rc_ref_pitch (setq _rc_ref_pitch __rc_ref_pitch)) _rc_ref_pitch)
  (:rc_ref_roll
   (&optional __rc_ref_roll)
   (if __rc_ref_roll (setq _rc_ref_roll __rc_ref_roll)) _rc_ref_roll)
  (:rc_ref_yaw
   (&optional __rc_ref_yaw)
   (if __rc_ref_yaw (setq _rc_ref_yaw __rc_ref_yaw)) _rc_ref_yaw)
  (:rc_ref_gaz
   (&optional __rc_ref_gaz)
   (if __rc_ref_gaz (setq _rc_ref_gaz __rc_ref_gaz)) _rc_ref_gaz)
  (:rc_ref_ag
   (&optional __rc_ref_ag)
   (if __rc_ref_ag (setq _rc_ref_ag __rc_ref_ag)) _rc_ref_ag)
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
    ;; int32 _rc_ref_pitch
    4
    ;; int32 _rc_ref_roll
    4
    ;; int32 _rc_ref_yaw
    4
    ;; int32 _rc_ref_gaz
    4
    ;; int32 _rc_ref_ag
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
     ;; int32 _rc_ref_pitch
       (write-long _rc_ref_pitch s)
     ;; int32 _rc_ref_roll
       (write-long _rc_ref_roll s)
     ;; int32 _rc_ref_yaw
       (write-long _rc_ref_yaw s)
     ;; int32 _rc_ref_gaz
       (write-long _rc_ref_gaz s)
     ;; int32 _rc_ref_ag
       (write-long _rc_ref_ag s)
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
   ;; int32 _rc_ref_pitch
     (setq _rc_ref_pitch (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _rc_ref_roll
     (setq _rc_ref_roll (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _rc_ref_yaw
     (setq _rc_ref_yaw (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _rc_ref_gaz
     (setq _rc_ref_gaz (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _rc_ref_ag
     (setq _rc_ref_ag (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get ardrone_autonomy::navdata_rc_references :md5sum-) "57eebe7ce3b678ed3cfbdcf4646ffe93")
(setf (get ardrone_autonomy::navdata_rc_references :datatype-) "ardrone_autonomy/navdata_rc_references")
(setf (get ardrone_autonomy::navdata_rc_references :definition-)
      "Header header
float64 drone_time
uint16 tag
uint16 size
int32 rc_ref_pitch
int32 rc_ref_roll
int32 rc_ref_yaw
int32 rc_ref_gaz
int32 rc_ref_ag

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



(provide :ardrone_autonomy/navdata_rc_references "57eebe7ce3b678ed3cfbdcf4646ffe93")


