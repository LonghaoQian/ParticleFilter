;; Auto-generated. Do not edit!


(when (boundp 'ardrone_autonomy::navdata_vision_detect)
  (if (not (find-package "ARDRONE_AUTONOMY"))
    (make-package "ARDRONE_AUTONOMY"))
  (shadow 'navdata_vision_detect (find-package "ARDRONE_AUTONOMY")))
(unless (find-package "ARDRONE_AUTONOMY::NAVDATA_VISION_DETECT")
  (make-package "ARDRONE_AUTONOMY::NAVDATA_VISION_DETECT"))

(in-package "ROS")
;;//! \htmlinclude navdata_vision_detect.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ardrone_autonomy::navdata_vision_detect
  :super ros::object
  :slots (_header _drone_time _tag _size _nb_detected _type _xc _yc _width _height _dist _orientation_angle _rotation _translation _camera_source ))

(defmethod ardrone_autonomy::navdata_vision_detect
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:drone_time __drone_time) 0.0)
    ((:tag __tag) 0)
    ((:size __size) 0)
    ((:nb_detected __nb_detected) 0)
    ((:type __type) (make-array 0 :initial-element 0 :element-type :integer))
    ((:xc __xc) (make-array 0 :initial-element 0 :element-type :integer))
    ((:yc __yc) (make-array 0 :initial-element 0 :element-type :integer))
    ((:width __width) (make-array 0 :initial-element 0 :element-type :integer))
    ((:height __height) (make-array 0 :initial-element 0 :element-type :integer))
    ((:dist __dist) (make-array 0 :initial-element 0 :element-type :integer))
    ((:orientation_angle __orientation_angle) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:rotation __rotation) (let (r) (dotimes (i 0) (push (instance ardrone_autonomy::matrix33 :init) r)) r))
    ((:translation __translation) (let (r) (dotimes (i 0) (push (instance ardrone_autonomy::vector31 :init) r)) r))
    ((:camera_source __camera_source) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _header __header)
   (setq _drone_time (float __drone_time))
   (setq _tag (round __tag))
   (setq _size (round __size))
   (setq _nb_detected (round __nb_detected))
   (setq _type __type)
   (setq _xc __xc)
   (setq _yc __yc)
   (setq _width __width)
   (setq _height __height)
   (setq _dist __dist)
   (setq _orientation_angle __orientation_angle)
   (setq _rotation __rotation)
   (setq _translation __translation)
   (setq _camera_source __camera_source)
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
  (:nb_detected
   (&optional __nb_detected)
   (if __nb_detected (setq _nb_detected __nb_detected)) _nb_detected)
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:xc
   (&optional __xc)
   (if __xc (setq _xc __xc)) _xc)
  (:yc
   (&optional __yc)
   (if __yc (setq _yc __yc)) _yc)
  (:width
   (&optional __width)
   (if __width (setq _width __width)) _width)
  (:height
   (&optional __height)
   (if __height (setq _height __height)) _height)
  (:dist
   (&optional __dist)
   (if __dist (setq _dist __dist)) _dist)
  (:orientation_angle
   (&optional __orientation_angle)
   (if __orientation_angle (setq _orientation_angle __orientation_angle)) _orientation_angle)
  (:rotation
   (&rest __rotation)
   (if (keywordp (car __rotation))
       (send* _rotation __rotation)
     (progn
       (if __rotation (setq _rotation (car __rotation)))
       _rotation)))
  (:translation
   (&rest __translation)
   (if (keywordp (car __translation))
       (send* _translation __translation)
     (progn
       (if __translation (setq _translation (car __translation)))
       _translation)))
  (:camera_source
   (&optional __camera_source)
   (if __camera_source (setq _camera_source __camera_source)) _camera_source)
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
    ;; uint32 _nb_detected
    4
    ;; uint32[] _type
    (* 4    (length _type)) 4
    ;; uint32[] _xc
    (* 4    (length _xc)) 4
    ;; uint32[] _yc
    (* 4    (length _yc)) 4
    ;; uint32[] _width
    (* 4    (length _width)) 4
    ;; uint32[] _height
    (* 4    (length _height)) 4
    ;; uint32[] _dist
    (* 4    (length _dist)) 4
    ;; float32[] _orientation_angle
    (* 4    (length _orientation_angle)) 4
    ;; ardrone_autonomy/matrix33[] _rotation
    (apply #'+ (send-all _rotation :serialization-length)) 4
    ;; ardrone_autonomy/vector31[] _translation
    (apply #'+ (send-all _translation :serialization-length)) 4
    ;; uint32[] _camera_source
    (* 4    (length _camera_source)) 4
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
     ;; uint32 _nb_detected
       (write-long _nb_detected s)
     ;; uint32[] _type
     (write-long (length _type) s)
     (dotimes (i (length _type))
       (write-long (elt _type i) s)
       )
     ;; uint32[] _xc
     (write-long (length _xc) s)
     (dotimes (i (length _xc))
       (write-long (elt _xc i) s)
       )
     ;; uint32[] _yc
     (write-long (length _yc) s)
     (dotimes (i (length _yc))
       (write-long (elt _yc i) s)
       )
     ;; uint32[] _width
     (write-long (length _width) s)
     (dotimes (i (length _width))
       (write-long (elt _width i) s)
       )
     ;; uint32[] _height
     (write-long (length _height) s)
     (dotimes (i (length _height))
       (write-long (elt _height i) s)
       )
     ;; uint32[] _dist
     (write-long (length _dist) s)
     (dotimes (i (length _dist))
       (write-long (elt _dist i) s)
       )
     ;; float32[] _orientation_angle
     (write-long (length _orientation_angle) s)
     (dotimes (i (length _orientation_angle))
       (sys::poke (elt _orientation_angle i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; ardrone_autonomy/matrix33[] _rotation
     (write-long (length _rotation) s)
     (dolist (elem _rotation)
       (send elem :serialize s)
       )
     ;; ardrone_autonomy/vector31[] _translation
     (write-long (length _translation) s)
     (dolist (elem _translation)
       (send elem :serialize s)
       )
     ;; uint32[] _camera_source
     (write-long (length _camera_source) s)
     (dotimes (i (length _camera_source))
       (write-long (elt _camera_source i) s)
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
   ;; uint32 _nb_detected
     (setq _nb_detected (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32[] _type
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _type (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _type i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; uint32[] _xc
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _xc (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _xc i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; uint32[] _yc
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _yc (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _yc i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; uint32[] _width
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _width (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _width i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; uint32[] _height
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _height (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _height i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; uint32[] _dist
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _dist (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _dist i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; float32[] _orientation_angle
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _orientation_angle (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _orientation_angle i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; ardrone_autonomy/matrix33[] _rotation
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _rotation (let (r) (dotimes (i n) (push (instance ardrone_autonomy::matrix33 :init) r)) r))
     (dolist (elem- _rotation)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; ardrone_autonomy/vector31[] _translation
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _translation (let (r) (dotimes (i n) (push (instance ardrone_autonomy::vector31 :init) r)) r))
     (dolist (elem- _translation)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; uint32[] _camera_source
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _camera_source (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _camera_source i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get ardrone_autonomy::navdata_vision_detect :md5sum-) "ab5f6b8c66aead58358d45a7c2dce7f6")
(setf (get ardrone_autonomy::navdata_vision_detect :datatype-) "ardrone_autonomy/navdata_vision_detect")
(setf (get ardrone_autonomy::navdata_vision_detect :definition-)
      "Header header
float64 drone_time
uint16 tag
uint16 size
uint32 nb_detected
uint32[] type
uint32[] xc
uint32[] yc
uint32[] width
uint32[] height
uint32[] dist
float32[] orientation_angle
matrix33[] rotation
vector31[] translation
uint32[] camera_source

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
MSG: ardrone_autonomy/matrix33
float32 m11
float32 m12
float32 m13
float32 m21
float32 m22
float32 m23
float32 m31
float32 m32
float32 m33
================================================================================
MSG: ardrone_autonomy/vector31
float32 x
float32 y
float32 z
")



(provide :ardrone_autonomy/navdata_vision_detect "ab5f6b8c66aead58358d45a7c2dce7f6")


