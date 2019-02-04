;; Auto-generated. Do not edit!


(when (boundp 'ardrone_autonomy::navdata_zimmu_3000)
  (if (not (find-package "ARDRONE_AUTONOMY"))
    (make-package "ARDRONE_AUTONOMY"))
  (shadow 'navdata_zimmu_3000 (find-package "ARDRONE_AUTONOMY")))
(unless (find-package "ARDRONE_AUTONOMY::NAVDATA_ZIMMU_3000")
  (make-package "ARDRONE_AUTONOMY::NAVDATA_ZIMMU_3000"))

(in-package "ROS")
;;//! \htmlinclude navdata_zimmu_3000.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ardrone_autonomy::navdata_zimmu_3000
  :super ros::object
  :slots (_header _drone_time _tag _size _vzimmuLSB _vzfind ))

(defmethod ardrone_autonomy::navdata_zimmu_3000
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:drone_time __drone_time) 0.0)
    ((:tag __tag) 0)
    ((:size __size) 0)
    ((:vzimmuLSB __vzimmuLSB) 0)
    ((:vzfind __vzfind) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _drone_time (float __drone_time))
   (setq _tag (round __tag))
   (setq _size (round __size))
   (setq _vzimmuLSB (round __vzimmuLSB))
   (setq _vzfind (float __vzfind))
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
  (:vzimmuLSB
   (&optional __vzimmuLSB)
   (if __vzimmuLSB (setq _vzimmuLSB __vzimmuLSB)) _vzimmuLSB)
  (:vzfind
   (&optional __vzfind)
   (if __vzfind (setq _vzfind __vzfind)) _vzfind)
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
    ;; int32 _vzimmuLSB
    4
    ;; float32 _vzfind
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
     ;; int32 _vzimmuLSB
       (write-long _vzimmuLSB s)
     ;; float32 _vzfind
       (sys::poke _vzfind (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
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
   ;; int32 _vzimmuLSB
     (setq _vzimmuLSB (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _vzfind
     (setq _vzfind (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get ardrone_autonomy::navdata_zimmu_3000 :md5sum-) "ae43ce2a4ef6cf0002f177f8265000bc")
(setf (get ardrone_autonomy::navdata_zimmu_3000 :datatype-) "ardrone_autonomy/navdata_zimmu_3000")
(setf (get ardrone_autonomy::navdata_zimmu_3000 :definition-)
      "Header header
float64 drone_time
uint16 tag
uint16 size
int32 vzimmuLSB
float32 vzfind

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



(provide :ardrone_autonomy/navdata_zimmu_3000 "ae43ce2a4ef6cf0002f177f8265000bc")


