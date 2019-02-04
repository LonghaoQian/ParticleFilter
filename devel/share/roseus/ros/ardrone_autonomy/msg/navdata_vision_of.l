;; Auto-generated. Do not edit!


(when (boundp 'ardrone_autonomy::navdata_vision_of)
  (if (not (find-package "ARDRONE_AUTONOMY"))
    (make-package "ARDRONE_AUTONOMY"))
  (shadow 'navdata_vision_of (find-package "ARDRONE_AUTONOMY")))
(unless (find-package "ARDRONE_AUTONOMY::NAVDATA_VISION_OF")
  (make-package "ARDRONE_AUTONOMY::NAVDATA_VISION_OF"))

(in-package "ROS")
;;//! \htmlinclude navdata_vision_of.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ardrone_autonomy::navdata_vision_of
  :super ros::object
  :slots (_header _drone_time _tag _size _of_dx _of_dy ))

(defmethod ardrone_autonomy::navdata_vision_of
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:drone_time __drone_time) 0.0)
    ((:tag __tag) 0)
    ((:size __size) 0)
    ((:of_dx __of_dx) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:of_dy __of_dy) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _drone_time (float __drone_time))
   (setq _tag (round __tag))
   (setq _size (round __size))
   (setq _of_dx __of_dx)
   (setq _of_dy __of_dy)
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
  (:of_dx
   (&optional __of_dx)
   (if __of_dx (setq _of_dx __of_dx)) _of_dx)
  (:of_dy
   (&optional __of_dy)
   (if __of_dy (setq _of_dy __of_dy)) _of_dy)
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
    ;; float32[] _of_dx
    (* 4    (length _of_dx)) 4
    ;; float32[] _of_dy
    (* 4    (length _of_dy)) 4
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
     ;; float32[] _of_dx
     (write-long (length _of_dx) s)
     (dotimes (i (length _of_dx))
       (sys::poke (elt _of_dx i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _of_dy
     (write-long (length _of_dy) s)
     (dotimes (i (length _of_dy))
       (sys::poke (elt _of_dy i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
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
   ;; float32[] _of_dx
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _of_dx (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _of_dx i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _of_dy
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _of_dy (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _of_dy i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get ardrone_autonomy::navdata_vision_of :md5sum-) "76d31747173a842fbf71db03104edecd")
(setf (get ardrone_autonomy::navdata_vision_of :datatype-) "ardrone_autonomy/navdata_vision_of")
(setf (get ardrone_autonomy::navdata_vision_of :definition-)
      "Header header
float64 drone_time
uint16 tag
uint16 size
float32[] of_dx
float32[] of_dy

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



(provide :ardrone_autonomy/navdata_vision_of "76d31747173a842fbf71db03104edecd")


