;; Auto-generated. Do not edit!


(when (boundp 'ardrone_autonomy::navdata_hdvideo_stream)
  (if (not (find-package "ARDRONE_AUTONOMY"))
    (make-package "ARDRONE_AUTONOMY"))
  (shadow 'navdata_hdvideo_stream (find-package "ARDRONE_AUTONOMY")))
(unless (find-package "ARDRONE_AUTONOMY::NAVDATA_HDVIDEO_STREAM")
  (make-package "ARDRONE_AUTONOMY::NAVDATA_HDVIDEO_STREAM"))

(in-package "ROS")
;;//! \htmlinclude navdata_hdvideo_stream.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ardrone_autonomy::navdata_hdvideo_stream
  :super ros::object
  :slots (_header _drone_time _tag _size _hdvideo_state _storage_fifo_nb_packets _storage_fifo_size _usbkey_size _usbkey_freespace _frame_number _usbkey_remaining_time ))

(defmethod ardrone_autonomy::navdata_hdvideo_stream
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:drone_time __drone_time) 0.0)
    ((:tag __tag) 0)
    ((:size __size) 0)
    ((:hdvideo_state __hdvideo_state) 0)
    ((:storage_fifo_nb_packets __storage_fifo_nb_packets) 0)
    ((:storage_fifo_size __storage_fifo_size) 0)
    ((:usbkey_size __usbkey_size) 0)
    ((:usbkey_freespace __usbkey_freespace) 0)
    ((:frame_number __frame_number) 0)
    ((:usbkey_remaining_time __usbkey_remaining_time) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _drone_time (float __drone_time))
   (setq _tag (round __tag))
   (setq _size (round __size))
   (setq _hdvideo_state (round __hdvideo_state))
   (setq _storage_fifo_nb_packets (round __storage_fifo_nb_packets))
   (setq _storage_fifo_size (round __storage_fifo_size))
   (setq _usbkey_size (round __usbkey_size))
   (setq _usbkey_freespace (round __usbkey_freespace))
   (setq _frame_number (round __frame_number))
   (setq _usbkey_remaining_time (round __usbkey_remaining_time))
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
  (:hdvideo_state
   (&optional __hdvideo_state)
   (if __hdvideo_state (setq _hdvideo_state __hdvideo_state)) _hdvideo_state)
  (:storage_fifo_nb_packets
   (&optional __storage_fifo_nb_packets)
   (if __storage_fifo_nb_packets (setq _storage_fifo_nb_packets __storage_fifo_nb_packets)) _storage_fifo_nb_packets)
  (:storage_fifo_size
   (&optional __storage_fifo_size)
   (if __storage_fifo_size (setq _storage_fifo_size __storage_fifo_size)) _storage_fifo_size)
  (:usbkey_size
   (&optional __usbkey_size)
   (if __usbkey_size (setq _usbkey_size __usbkey_size)) _usbkey_size)
  (:usbkey_freespace
   (&optional __usbkey_freespace)
   (if __usbkey_freespace (setq _usbkey_freespace __usbkey_freespace)) _usbkey_freespace)
  (:frame_number
   (&optional __frame_number)
   (if __frame_number (setq _frame_number __frame_number)) _frame_number)
  (:usbkey_remaining_time
   (&optional __usbkey_remaining_time)
   (if __usbkey_remaining_time (setq _usbkey_remaining_time __usbkey_remaining_time)) _usbkey_remaining_time)
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
    ;; uint32 _hdvideo_state
    4
    ;; uint32 _storage_fifo_nb_packets
    4
    ;; uint32 _storage_fifo_size
    4
    ;; uint32 _usbkey_size
    4
    ;; uint32 _usbkey_freespace
    4
    ;; uint32 _frame_number
    4
    ;; uint32 _usbkey_remaining_time
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
     ;; uint32 _hdvideo_state
       (write-long _hdvideo_state s)
     ;; uint32 _storage_fifo_nb_packets
       (write-long _storage_fifo_nb_packets s)
     ;; uint32 _storage_fifo_size
       (write-long _storage_fifo_size s)
     ;; uint32 _usbkey_size
       (write-long _usbkey_size s)
     ;; uint32 _usbkey_freespace
       (write-long _usbkey_freespace s)
     ;; uint32 _frame_number
       (write-long _frame_number s)
     ;; uint32 _usbkey_remaining_time
       (write-long _usbkey_remaining_time s)
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
   ;; uint32 _hdvideo_state
     (setq _hdvideo_state (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _storage_fifo_nb_packets
     (setq _storage_fifo_nb_packets (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _storage_fifo_size
     (setq _storage_fifo_size (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _usbkey_size
     (setq _usbkey_size (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _usbkey_freespace
     (setq _usbkey_freespace (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _frame_number
     (setq _frame_number (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _usbkey_remaining_time
     (setq _usbkey_remaining_time (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get ardrone_autonomy::navdata_hdvideo_stream :md5sum-) "1ba321578916df95f899ca2f5348f234")
(setf (get ardrone_autonomy::navdata_hdvideo_stream :datatype-) "ardrone_autonomy/navdata_hdvideo_stream")
(setf (get ardrone_autonomy::navdata_hdvideo_stream :definition-)
      "Header header
float64 drone_time
uint16 tag
uint16 size
uint32 hdvideo_state
uint32 storage_fifo_nb_packets
uint32 storage_fifo_size
uint32 usbkey_size
uint32 usbkey_freespace
uint32 frame_number
uint32 usbkey_remaining_time

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



(provide :ardrone_autonomy/navdata_hdvideo_stream "1ba321578916df95f899ca2f5348f234")


