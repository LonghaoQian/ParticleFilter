;; Auto-generated. Do not edit!


(when (boundp 'ardrone_autonomy::navdata_adc_data_frame)
  (if (not (find-package "ARDRONE_AUTONOMY"))
    (make-package "ARDRONE_AUTONOMY"))
  (shadow 'navdata_adc_data_frame (find-package "ARDRONE_AUTONOMY")))
(unless (find-package "ARDRONE_AUTONOMY::NAVDATA_ADC_DATA_FRAME")
  (make-package "ARDRONE_AUTONOMY::NAVDATA_ADC_DATA_FRAME"))

(in-package "ROS")
;;//! \htmlinclude navdata_adc_data_frame.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ardrone_autonomy::navdata_adc_data_frame
  :super ros::object
  :slots (_header _drone_time _tag _size _version _data_frame ))

(defmethod ardrone_autonomy::navdata_adc_data_frame
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:drone_time __drone_time) 0.0)
    ((:tag __tag) 0)
    ((:size __size) 0)
    ((:version __version) 0)
    ((:data_frame __data_frame) (make-array 0 :initial-element 0 :element-type :char))
    )
   (send-super :init)
   (setq _header __header)
   (setq _drone_time (float __drone_time))
   (setq _tag (round __tag))
   (setq _size (round __size))
   (setq _version (round __version))
   (setq _data_frame __data_frame)
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
  (:version
   (&optional __version)
   (if __version (setq _version __version)) _version)
  (:data_frame
   (&optional __data_frame)
   (if __data_frame (setq _data_frame __data_frame)) _data_frame)
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
    ;; uint32 _version
    4
    ;; uint8[] _data_frame
    (* 1    (length _data_frame)) 4
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
     ;; uint32 _version
       (write-long _version s)
     ;; uint8[] _data_frame
     (write-long (length _data_frame) s)
     (princ _data_frame s)
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
   ;; uint32 _version
     (setq _version (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint8[] _data_frame
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _data_frame (make-array n :element-type :char))
     (replace _data_frame buf :start2 ptr-) (incf ptr- n))
   ;;
   self)
  )

(setf (get ardrone_autonomy::navdata_adc_data_frame :md5sum-) "403dbf67522d768c3f509d9614c23941")
(setf (get ardrone_autonomy::navdata_adc_data_frame :datatype-) "ardrone_autonomy/navdata_adc_data_frame")
(setf (get ardrone_autonomy::navdata_adc_data_frame :definition-)
      "Header header
float64 drone_time
uint16 tag
uint16 size
uint32 version
uint8[] data_frame

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



(provide :ardrone_autonomy/navdata_adc_data_frame "403dbf67522d768c3f509d9614c23941")


