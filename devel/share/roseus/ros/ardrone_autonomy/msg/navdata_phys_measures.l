;; Auto-generated. Do not edit!


(when (boundp 'ardrone_autonomy::navdata_phys_measures)
  (if (not (find-package "ARDRONE_AUTONOMY"))
    (make-package "ARDRONE_AUTONOMY"))
  (shadow 'navdata_phys_measures (find-package "ARDRONE_AUTONOMY")))
(unless (find-package "ARDRONE_AUTONOMY::NAVDATA_PHYS_MEASURES")
  (make-package "ARDRONE_AUTONOMY::NAVDATA_PHYS_MEASURES"))

(in-package "ROS")
;;//! \htmlinclude navdata_phys_measures.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ardrone_autonomy::navdata_phys_measures
  :super ros::object
  :slots (_header _drone_time _tag _size _accs_temp _gyro_temp _phys_accs _phys_gyros _alim3V3 _vrefEpson _vrefIDG ))

(defmethod ardrone_autonomy::navdata_phys_measures
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:drone_time __drone_time) 0.0)
    ((:tag __tag) 0)
    ((:size __size) 0)
    ((:accs_temp __accs_temp) 0.0)
    ((:gyro_temp __gyro_temp) 0)
    ((:phys_accs __phys_accs) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:phys_gyros __phys_gyros) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:alim3V3 __alim3V3) 0)
    ((:vrefEpson __vrefEpson) 0)
    ((:vrefIDG __vrefIDG) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _drone_time (float __drone_time))
   (setq _tag (round __tag))
   (setq _size (round __size))
   (setq _accs_temp (float __accs_temp))
   (setq _gyro_temp (round __gyro_temp))
   (setq _phys_accs __phys_accs)
   (setq _phys_gyros __phys_gyros)
   (setq _alim3V3 (round __alim3V3))
   (setq _vrefEpson (round __vrefEpson))
   (setq _vrefIDG (round __vrefIDG))
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
  (:accs_temp
   (&optional __accs_temp)
   (if __accs_temp (setq _accs_temp __accs_temp)) _accs_temp)
  (:gyro_temp
   (&optional __gyro_temp)
   (if __gyro_temp (setq _gyro_temp __gyro_temp)) _gyro_temp)
  (:phys_accs
   (&optional __phys_accs)
   (if __phys_accs (setq _phys_accs __phys_accs)) _phys_accs)
  (:phys_gyros
   (&optional __phys_gyros)
   (if __phys_gyros (setq _phys_gyros __phys_gyros)) _phys_gyros)
  (:alim3V3
   (&optional __alim3V3)
   (if __alim3V3 (setq _alim3V3 __alim3V3)) _alim3V3)
  (:vrefEpson
   (&optional __vrefEpson)
   (if __vrefEpson (setq _vrefEpson __vrefEpson)) _vrefEpson)
  (:vrefIDG
   (&optional __vrefIDG)
   (if __vrefIDG (setq _vrefIDG __vrefIDG)) _vrefIDG)
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
    ;; float32 _accs_temp
    4
    ;; uint16 _gyro_temp
    2
    ;; float32[] _phys_accs
    (* 4    (length _phys_accs)) 4
    ;; float32[] _phys_gyros
    (* 4    (length _phys_gyros)) 4
    ;; uint32 _alim3V3
    4
    ;; uint32 _vrefEpson
    4
    ;; uint32 _vrefIDG
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
     ;; float32 _accs_temp
       (sys::poke _accs_temp (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint16 _gyro_temp
       (write-word _gyro_temp s)
     ;; float32[] _phys_accs
     (write-long (length _phys_accs) s)
     (dotimes (i (length _phys_accs))
       (sys::poke (elt _phys_accs i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _phys_gyros
     (write-long (length _phys_gyros) s)
     (dotimes (i (length _phys_gyros))
       (sys::poke (elt _phys_gyros i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; uint32 _alim3V3
       (write-long _alim3V3 s)
     ;; uint32 _vrefEpson
       (write-long _vrefEpson s)
     ;; uint32 _vrefIDG
       (write-long _vrefIDG s)
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
   ;; float32 _accs_temp
     (setq _accs_temp (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint16 _gyro_temp
     (setq _gyro_temp (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float32[] _phys_accs
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _phys_accs (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _phys_accs i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _phys_gyros
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _phys_gyros (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _phys_gyros i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; uint32 _alim3V3
     (setq _alim3V3 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _vrefEpson
     (setq _vrefEpson (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _vrefIDG
     (setq _vrefIDG (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get ardrone_autonomy::navdata_phys_measures :md5sum-) "649dd469618f5ff313f6bceab2c356b7")
(setf (get ardrone_autonomy::navdata_phys_measures :datatype-) "ardrone_autonomy/navdata_phys_measures")
(setf (get ardrone_autonomy::navdata_phys_measures :definition-)
      "Header header
float64 drone_time
uint16 tag
uint16 size
float32 accs_temp
uint16 gyro_temp
float32[] phys_accs
float32[] phys_gyros
uint32 alim3V3
uint32 vrefEpson
uint32 vrefIDG

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



(provide :ardrone_autonomy/navdata_phys_measures "649dd469618f5ff313f6bceab2c356b7")


