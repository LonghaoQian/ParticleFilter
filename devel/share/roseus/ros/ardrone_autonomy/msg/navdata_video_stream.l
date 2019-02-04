;; Auto-generated. Do not edit!


(when (boundp 'ardrone_autonomy::navdata_video_stream)
  (if (not (find-package "ARDRONE_AUTONOMY"))
    (make-package "ARDRONE_AUTONOMY"))
  (shadow 'navdata_video_stream (find-package "ARDRONE_AUTONOMY")))
(unless (find-package "ARDRONE_AUTONOMY::NAVDATA_VIDEO_STREAM")
  (make-package "ARDRONE_AUTONOMY::NAVDATA_VIDEO_STREAM"))

(in-package "ROS")
;;//! \htmlinclude navdata_video_stream.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ardrone_autonomy::navdata_video_stream
  :super ros::object
  :slots (_header _drone_time _tag _size _quant _frame_size _frame_number _atcmd_ref_seq _atcmd_mean_ref_gap _atcmd_var_ref_gap _atcmd_ref_quality _desired_bitrate _data2 _data3 _data4 _data5 _fifo_queue_level ))

(defmethod ardrone_autonomy::navdata_video_stream
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:drone_time __drone_time) 0.0)
    ((:tag __tag) 0)
    ((:size __size) 0)
    ((:quant __quant) 0)
    ((:frame_size __frame_size) 0)
    ((:frame_number __frame_number) 0)
    ((:atcmd_ref_seq __atcmd_ref_seq) 0)
    ((:atcmd_mean_ref_gap __atcmd_mean_ref_gap) 0)
    ((:atcmd_var_ref_gap __atcmd_var_ref_gap) 0.0)
    ((:atcmd_ref_quality __atcmd_ref_quality) 0)
    ((:desired_bitrate __desired_bitrate) 0)
    ((:data2 __data2) 0)
    ((:data3 __data3) 0)
    ((:data4 __data4) 0)
    ((:data5 __data5) 0)
    ((:fifo_queue_level __fifo_queue_level) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _drone_time (float __drone_time))
   (setq _tag (round __tag))
   (setq _size (round __size))
   (setq _quant (round __quant))
   (setq _frame_size (round __frame_size))
   (setq _frame_number (round __frame_number))
   (setq _atcmd_ref_seq (round __atcmd_ref_seq))
   (setq _atcmd_mean_ref_gap (round __atcmd_mean_ref_gap))
   (setq _atcmd_var_ref_gap (float __atcmd_var_ref_gap))
   (setq _atcmd_ref_quality (round __atcmd_ref_quality))
   (setq _desired_bitrate (round __desired_bitrate))
   (setq _data2 (round __data2))
   (setq _data3 (round __data3))
   (setq _data4 (round __data4))
   (setq _data5 (round __data5))
   (setq _fifo_queue_level (round __fifo_queue_level))
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
  (:quant
   (&optional __quant)
   (if __quant (setq _quant __quant)) _quant)
  (:frame_size
   (&optional __frame_size)
   (if __frame_size (setq _frame_size __frame_size)) _frame_size)
  (:frame_number
   (&optional __frame_number)
   (if __frame_number (setq _frame_number __frame_number)) _frame_number)
  (:atcmd_ref_seq
   (&optional __atcmd_ref_seq)
   (if __atcmd_ref_seq (setq _atcmd_ref_seq __atcmd_ref_seq)) _atcmd_ref_seq)
  (:atcmd_mean_ref_gap
   (&optional __atcmd_mean_ref_gap)
   (if __atcmd_mean_ref_gap (setq _atcmd_mean_ref_gap __atcmd_mean_ref_gap)) _atcmd_mean_ref_gap)
  (:atcmd_var_ref_gap
   (&optional __atcmd_var_ref_gap)
   (if __atcmd_var_ref_gap (setq _atcmd_var_ref_gap __atcmd_var_ref_gap)) _atcmd_var_ref_gap)
  (:atcmd_ref_quality
   (&optional __atcmd_ref_quality)
   (if __atcmd_ref_quality (setq _atcmd_ref_quality __atcmd_ref_quality)) _atcmd_ref_quality)
  (:desired_bitrate
   (&optional __desired_bitrate)
   (if __desired_bitrate (setq _desired_bitrate __desired_bitrate)) _desired_bitrate)
  (:data2
   (&optional __data2)
   (if __data2 (setq _data2 __data2)) _data2)
  (:data3
   (&optional __data3)
   (if __data3 (setq _data3 __data3)) _data3)
  (:data4
   (&optional __data4)
   (if __data4 (setq _data4 __data4)) _data4)
  (:data5
   (&optional __data5)
   (if __data5 (setq _data5 __data5)) _data5)
  (:fifo_queue_level
   (&optional __fifo_queue_level)
   (if __fifo_queue_level (setq _fifo_queue_level __fifo_queue_level)) _fifo_queue_level)
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
    ;; uint8 _quant
    1
    ;; uint32 _frame_size
    4
    ;; uint32 _frame_number
    4
    ;; uint32 _atcmd_ref_seq
    4
    ;; uint32 _atcmd_mean_ref_gap
    4
    ;; float32 _atcmd_var_ref_gap
    4
    ;; uint32 _atcmd_ref_quality
    4
    ;; uint32 _desired_bitrate
    4
    ;; int32 _data2
    4
    ;; int32 _data3
    4
    ;; int32 _data4
    4
    ;; int32 _data5
    4
    ;; uint32 _fifo_queue_level
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
     ;; uint8 _quant
       (write-byte _quant s)
     ;; uint32 _frame_size
       (write-long _frame_size s)
     ;; uint32 _frame_number
       (write-long _frame_number s)
     ;; uint32 _atcmd_ref_seq
       (write-long _atcmd_ref_seq s)
     ;; uint32 _atcmd_mean_ref_gap
       (write-long _atcmd_mean_ref_gap s)
     ;; float32 _atcmd_var_ref_gap
       (sys::poke _atcmd_var_ref_gap (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint32 _atcmd_ref_quality
       (write-long _atcmd_ref_quality s)
     ;; uint32 _desired_bitrate
       (write-long _desired_bitrate s)
     ;; int32 _data2
       (write-long _data2 s)
     ;; int32 _data3
       (write-long _data3 s)
     ;; int32 _data4
       (write-long _data4 s)
     ;; int32 _data5
       (write-long _data5 s)
     ;; uint32 _fifo_queue_level
       (write-long _fifo_queue_level s)
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
   ;; uint8 _quant
     (setq _quant (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint32 _frame_size
     (setq _frame_size (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _frame_number
     (setq _frame_number (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _atcmd_ref_seq
     (setq _atcmd_ref_seq (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _atcmd_mean_ref_gap
     (setq _atcmd_mean_ref_gap (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _atcmd_var_ref_gap
     (setq _atcmd_var_ref_gap (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint32 _atcmd_ref_quality
     (setq _atcmd_ref_quality (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _desired_bitrate
     (setq _desired_bitrate (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _data2
     (setq _data2 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _data3
     (setq _data3 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _data4
     (setq _data4 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _data5
     (setq _data5 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _fifo_queue_level
     (setq _fifo_queue_level (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get ardrone_autonomy::navdata_video_stream :md5sum-) "caf560356d435eb563ce5b9e8278da00")
(setf (get ardrone_autonomy::navdata_video_stream :datatype-) "ardrone_autonomy/navdata_video_stream")
(setf (get ardrone_autonomy::navdata_video_stream :definition-)
      "Header header
float64 drone_time
uint16 tag
uint16 size
uint8 quant
uint32 frame_size
uint32 frame_number
uint32 atcmd_ref_seq
uint32 atcmd_mean_ref_gap
float32 atcmd_var_ref_gap
uint32 atcmd_ref_quality
uint32 desired_bitrate
int32 data2
int32 data3
int32 data4
int32 data5
uint32 fifo_queue_level

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



(provide :ardrone_autonomy/navdata_video_stream "caf560356d435eb563ce5b9e8278da00")


