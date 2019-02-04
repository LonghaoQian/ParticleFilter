;; Auto-generated. Do not edit!


(when (boundp 'ardrone_autonomy::navdata_trackers_send)
  (if (not (find-package "ARDRONE_AUTONOMY"))
    (make-package "ARDRONE_AUTONOMY"))
  (shadow 'navdata_trackers_send (find-package "ARDRONE_AUTONOMY")))
(unless (find-package "ARDRONE_AUTONOMY::NAVDATA_TRACKERS_SEND")
  (make-package "ARDRONE_AUTONOMY::NAVDATA_TRACKERS_SEND"))

(in-package "ROS")
;;//! \htmlinclude navdata_trackers_send.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ardrone_autonomy::navdata_trackers_send
  :super ros::object
  :slots (_header _drone_time _tag _size _locked _point ))

(defmethod ardrone_autonomy::navdata_trackers_send
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:drone_time __drone_time) 0.0)
    ((:tag __tag) 0)
    ((:size __size) 0)
    ((:locked __locked) (make-array 0 :initial-element 0 :element-type :integer))
    ((:point __point) (let (r) (dotimes (i 0) (push (instance ardrone_autonomy::vector21 :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _drone_time (float __drone_time))
   (setq _tag (round __tag))
   (setq _size (round __size))
   (setq _locked __locked)
   (setq _point __point)
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
  (:locked
   (&optional __locked)
   (if __locked (setq _locked __locked)) _locked)
  (:point
   (&rest __point)
   (if (keywordp (car __point))
       (send* _point __point)
     (progn
       (if __point (setq _point (car __point)))
       _point)))
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
    ;; int32[] _locked
    (* 4    (length _locked)) 4
    ;; ardrone_autonomy/vector21[] _point
    (apply #'+ (send-all _point :serialization-length)) 4
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
     ;; int32[] _locked
     (write-long (length _locked) s)
     (dotimes (i (length _locked))
       (write-long (elt _locked i) s)
       )
     ;; ardrone_autonomy/vector21[] _point
     (write-long (length _point) s)
     (dolist (elem _point)
       (send elem :serialize s)
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
   ;; int32[] _locked
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _locked (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _locked i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; ardrone_autonomy/vector21[] _point
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _point (let (r) (dotimes (i n) (push (instance ardrone_autonomy::vector21 :init) r)) r))
     (dolist (elem- _point)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get ardrone_autonomy::navdata_trackers_send :md5sum-) "77326ae58ca7691bcd2f23425242a70e")
(setf (get ardrone_autonomy::navdata_trackers_send :datatype-) "ardrone_autonomy/navdata_trackers_send")
(setf (get ardrone_autonomy::navdata_trackers_send :definition-)
      "Header header
float64 drone_time
uint16 tag
uint16 size
int32[] locked
vector21[] point

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

================================================================================
MSG: ardrone_autonomy/vector21
float32 x
float32 y
")



(provide :ardrone_autonomy/navdata_trackers_send "77326ae58ca7691bcd2f23425242a70e")


