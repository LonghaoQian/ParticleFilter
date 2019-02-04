;; Auto-generated. Do not edit!


(when (boundp 'ardrone_autonomy::Navdata)
  (if (not (find-package "ARDRONE_AUTONOMY"))
    (make-package "ARDRONE_AUTONOMY"))
  (shadow 'Navdata (find-package "ARDRONE_AUTONOMY")))
(unless (find-package "ARDRONE_AUTONOMY::NAVDATA")
  (make-package "ARDRONE_AUTONOMY::NAVDATA"))

(in-package "ROS")
;;//! \htmlinclude Navdata.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ardrone_autonomy::Navdata
  :super ros::object
  :slots (_header _batteryPercent _state _magX _magY _magZ _pressure _temp _wind_speed _wind_angle _wind_comp_angle _rotX _rotY _rotZ _altd _vx _vy _vz _ax _ay _az _motor1 _motor2 _motor3 _motor4 _tags_count _tags_type _tags_xc _tags_yc _tags_width _tags_height _tags_orientation _tags_distance _tm ))

(defmethod ardrone_autonomy::Navdata
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:batteryPercent __batteryPercent) 0.0)
    ((:state __state) 0)
    ((:magX __magX) 0)
    ((:magY __magY) 0)
    ((:magZ __magZ) 0)
    ((:pressure __pressure) 0)
    ((:temp __temp) 0)
    ((:wind_speed __wind_speed) 0.0)
    ((:wind_angle __wind_angle) 0.0)
    ((:wind_comp_angle __wind_comp_angle) 0.0)
    ((:rotX __rotX) 0.0)
    ((:rotY __rotY) 0.0)
    ((:rotZ __rotZ) 0.0)
    ((:altd __altd) 0)
    ((:vx __vx) 0.0)
    ((:vy __vy) 0.0)
    ((:vz __vz) 0.0)
    ((:ax __ax) 0.0)
    ((:ay __ay) 0.0)
    ((:az __az) 0.0)
    ((:motor1 __motor1) 0)
    ((:motor2 __motor2) 0)
    ((:motor3 __motor3) 0)
    ((:motor4 __motor4) 0)
    ((:tags_count __tags_count) 0)
    ((:tags_type __tags_type) (make-array 0 :initial-element 0 :element-type :integer))
    ((:tags_xc __tags_xc) (make-array 0 :initial-element 0 :element-type :integer))
    ((:tags_yc __tags_yc) (make-array 0 :initial-element 0 :element-type :integer))
    ((:tags_width __tags_width) (make-array 0 :initial-element 0 :element-type :integer))
    ((:tags_height __tags_height) (make-array 0 :initial-element 0 :element-type :integer))
    ((:tags_orientation __tags_orientation) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:tags_distance __tags_distance) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:tm __tm) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _batteryPercent (float __batteryPercent))
   (setq _state (round __state))
   (setq _magX (round __magX))
   (setq _magY (round __magY))
   (setq _magZ (round __magZ))
   (setq _pressure (round __pressure))
   (setq _temp (round __temp))
   (setq _wind_speed (float __wind_speed))
   (setq _wind_angle (float __wind_angle))
   (setq _wind_comp_angle (float __wind_comp_angle))
   (setq _rotX (float __rotX))
   (setq _rotY (float __rotY))
   (setq _rotZ (float __rotZ))
   (setq _altd (round __altd))
   (setq _vx (float __vx))
   (setq _vy (float __vy))
   (setq _vz (float __vz))
   (setq _ax (float __ax))
   (setq _ay (float __ay))
   (setq _az (float __az))
   (setq _motor1 (round __motor1))
   (setq _motor2 (round __motor2))
   (setq _motor3 (round __motor3))
   (setq _motor4 (round __motor4))
   (setq _tags_count (round __tags_count))
   (setq _tags_type __tags_type)
   (setq _tags_xc __tags_xc)
   (setq _tags_yc __tags_yc)
   (setq _tags_width __tags_width)
   (setq _tags_height __tags_height)
   (setq _tags_orientation __tags_orientation)
   (setq _tags_distance __tags_distance)
   (setq _tm (float __tm))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:batteryPercent
   (&optional __batteryPercent)
   (if __batteryPercent (setq _batteryPercent __batteryPercent)) _batteryPercent)
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
  (:magX
   (&optional __magX)
   (if __magX (setq _magX __magX)) _magX)
  (:magY
   (&optional __magY)
   (if __magY (setq _magY __magY)) _magY)
  (:magZ
   (&optional __magZ)
   (if __magZ (setq _magZ __magZ)) _magZ)
  (:pressure
   (&optional __pressure)
   (if __pressure (setq _pressure __pressure)) _pressure)
  (:temp
   (&optional __temp)
   (if __temp (setq _temp __temp)) _temp)
  (:wind_speed
   (&optional __wind_speed)
   (if __wind_speed (setq _wind_speed __wind_speed)) _wind_speed)
  (:wind_angle
   (&optional __wind_angle)
   (if __wind_angle (setq _wind_angle __wind_angle)) _wind_angle)
  (:wind_comp_angle
   (&optional __wind_comp_angle)
   (if __wind_comp_angle (setq _wind_comp_angle __wind_comp_angle)) _wind_comp_angle)
  (:rotX
   (&optional __rotX)
   (if __rotX (setq _rotX __rotX)) _rotX)
  (:rotY
   (&optional __rotY)
   (if __rotY (setq _rotY __rotY)) _rotY)
  (:rotZ
   (&optional __rotZ)
   (if __rotZ (setq _rotZ __rotZ)) _rotZ)
  (:altd
   (&optional __altd)
   (if __altd (setq _altd __altd)) _altd)
  (:vx
   (&optional __vx)
   (if __vx (setq _vx __vx)) _vx)
  (:vy
   (&optional __vy)
   (if __vy (setq _vy __vy)) _vy)
  (:vz
   (&optional __vz)
   (if __vz (setq _vz __vz)) _vz)
  (:ax
   (&optional __ax)
   (if __ax (setq _ax __ax)) _ax)
  (:ay
   (&optional __ay)
   (if __ay (setq _ay __ay)) _ay)
  (:az
   (&optional __az)
   (if __az (setq _az __az)) _az)
  (:motor1
   (&optional __motor1)
   (if __motor1 (setq _motor1 __motor1)) _motor1)
  (:motor2
   (&optional __motor2)
   (if __motor2 (setq _motor2 __motor2)) _motor2)
  (:motor3
   (&optional __motor3)
   (if __motor3 (setq _motor3 __motor3)) _motor3)
  (:motor4
   (&optional __motor4)
   (if __motor4 (setq _motor4 __motor4)) _motor4)
  (:tags_count
   (&optional __tags_count)
   (if __tags_count (setq _tags_count __tags_count)) _tags_count)
  (:tags_type
   (&optional __tags_type)
   (if __tags_type (setq _tags_type __tags_type)) _tags_type)
  (:tags_xc
   (&optional __tags_xc)
   (if __tags_xc (setq _tags_xc __tags_xc)) _tags_xc)
  (:tags_yc
   (&optional __tags_yc)
   (if __tags_yc (setq _tags_yc __tags_yc)) _tags_yc)
  (:tags_width
   (&optional __tags_width)
   (if __tags_width (setq _tags_width __tags_width)) _tags_width)
  (:tags_height
   (&optional __tags_height)
   (if __tags_height (setq _tags_height __tags_height)) _tags_height)
  (:tags_orientation
   (&optional __tags_orientation)
   (if __tags_orientation (setq _tags_orientation __tags_orientation)) _tags_orientation)
  (:tags_distance
   (&optional __tags_distance)
   (if __tags_distance (setq _tags_distance __tags_distance)) _tags_distance)
  (:tm
   (&optional __tm)
   (if __tm (setq _tm __tm)) _tm)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32 _batteryPercent
    4
    ;; uint32 _state
    4
    ;; int32 _magX
    4
    ;; int32 _magY
    4
    ;; int32 _magZ
    4
    ;; int32 _pressure
    4
    ;; int32 _temp
    4
    ;; float32 _wind_speed
    4
    ;; float32 _wind_angle
    4
    ;; float32 _wind_comp_angle
    4
    ;; float32 _rotX
    4
    ;; float32 _rotY
    4
    ;; float32 _rotZ
    4
    ;; int32 _altd
    4
    ;; float32 _vx
    4
    ;; float32 _vy
    4
    ;; float32 _vz
    4
    ;; float32 _ax
    4
    ;; float32 _ay
    4
    ;; float32 _az
    4
    ;; uint8 _motor1
    1
    ;; uint8 _motor2
    1
    ;; uint8 _motor3
    1
    ;; uint8 _motor4
    1
    ;; uint32 _tags_count
    4
    ;; uint32[] _tags_type
    (* 4    (length _tags_type)) 4
    ;; uint32[] _tags_xc
    (* 4    (length _tags_xc)) 4
    ;; uint32[] _tags_yc
    (* 4    (length _tags_yc)) 4
    ;; uint32[] _tags_width
    (* 4    (length _tags_width)) 4
    ;; uint32[] _tags_height
    (* 4    (length _tags_height)) 4
    ;; float32[] _tags_orientation
    (* 4    (length _tags_orientation)) 4
    ;; float32[] _tags_distance
    (* 4    (length _tags_distance)) 4
    ;; float32 _tm
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32 _batteryPercent
       (sys::poke _batteryPercent (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint32 _state
       (write-long _state s)
     ;; int32 _magX
       (write-long _magX s)
     ;; int32 _magY
       (write-long _magY s)
     ;; int32 _magZ
       (write-long _magZ s)
     ;; int32 _pressure
       (write-long _pressure s)
     ;; int32 _temp
       (write-long _temp s)
     ;; float32 _wind_speed
       (sys::poke _wind_speed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _wind_angle
       (sys::poke _wind_angle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _wind_comp_angle
       (sys::poke _wind_comp_angle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _rotX
       (sys::poke _rotX (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _rotY
       (sys::poke _rotY (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _rotZ
       (sys::poke _rotZ (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32 _altd
       (write-long _altd s)
     ;; float32 _vx
       (sys::poke _vx (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _vy
       (sys::poke _vy (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _vz
       (sys::poke _vz (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _ax
       (sys::poke _ax (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _ay
       (sys::poke _ay (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _az
       (sys::poke _az (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint8 _motor1
       (write-byte _motor1 s)
     ;; uint8 _motor2
       (write-byte _motor2 s)
     ;; uint8 _motor3
       (write-byte _motor3 s)
     ;; uint8 _motor4
       (write-byte _motor4 s)
     ;; uint32 _tags_count
       (write-long _tags_count s)
     ;; uint32[] _tags_type
     (write-long (length _tags_type) s)
     (dotimes (i (length _tags_type))
       (write-long (elt _tags_type i) s)
       )
     ;; uint32[] _tags_xc
     (write-long (length _tags_xc) s)
     (dotimes (i (length _tags_xc))
       (write-long (elt _tags_xc i) s)
       )
     ;; uint32[] _tags_yc
     (write-long (length _tags_yc) s)
     (dotimes (i (length _tags_yc))
       (write-long (elt _tags_yc i) s)
       )
     ;; uint32[] _tags_width
     (write-long (length _tags_width) s)
     (dotimes (i (length _tags_width))
       (write-long (elt _tags_width i) s)
       )
     ;; uint32[] _tags_height
     (write-long (length _tags_height) s)
     (dotimes (i (length _tags_height))
       (write-long (elt _tags_height i) s)
       )
     ;; float32[] _tags_orientation
     (write-long (length _tags_orientation) s)
     (dotimes (i (length _tags_orientation))
       (sys::poke (elt _tags_orientation i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _tags_distance
     (write-long (length _tags_distance) s)
     (dotimes (i (length _tags_distance))
       (sys::poke (elt _tags_distance i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32 _tm
       (sys::poke _tm (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32 _batteryPercent
     (setq _batteryPercent (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint32 _state
     (setq _state (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _magX
     (setq _magX (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _magY
     (setq _magY (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _magZ
     (setq _magZ (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _pressure
     (setq _pressure (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _temp
     (setq _temp (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _wind_speed
     (setq _wind_speed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _wind_angle
     (setq _wind_angle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _wind_comp_angle
     (setq _wind_comp_angle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _rotX
     (setq _rotX (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _rotY
     (setq _rotY (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _rotZ
     (setq _rotZ (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32 _altd
     (setq _altd (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _vx
     (setq _vx (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _vy
     (setq _vy (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _vz
     (setq _vz (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _ax
     (setq _ax (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _ay
     (setq _ay (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _az
     (setq _az (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint8 _motor1
     (setq _motor1 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _motor2
     (setq _motor2 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _motor3
     (setq _motor3 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _motor4
     (setq _motor4 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint32 _tags_count
     (setq _tags_count (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32[] _tags_type
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _tags_type (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _tags_type i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; uint32[] _tags_xc
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _tags_xc (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _tags_xc i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; uint32[] _tags_yc
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _tags_yc (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _tags_yc i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; uint32[] _tags_width
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _tags_width (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _tags_width i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; uint32[] _tags_height
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _tags_height (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _tags_height i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; float32[] _tags_orientation
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _tags_orientation (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _tags_orientation i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _tags_distance
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _tags_distance (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _tags_distance i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32 _tm
     (setq _tm (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get ardrone_autonomy::Navdata :md5sum-) "e1169f766234363125ac62c9a3f87eeb")
(setf (get ardrone_autonomy::Navdata :datatype-) "ardrone_autonomy/Navdata")
(setf (get ardrone_autonomy::Navdata :definition-)
      "Header header

# Navdata including the ARDrone 2 specifica sensors
# (magnetometer, barometer)

# 0 means no battery, 100 means full battery
float32 batteryPercent

# 0: Unknown, 1: Init, 2: Landed, 3: Flying, 4: Hovering, 5: Test
# 6: Taking off, 7: Goto Fix Point, 8: Landing, 9: Looping
# Note: 3,7 seems to discriminate type of flying (isFly = 3 | 7)
uint32 state

int32 magX
int32 magY
int32 magZ

# pressure sensor
int32 pressure

# apparently, there was a temperature sensor added as well.
int32 temp

# wind sensing...
float32 wind_speed
float32 wind_angle
float32 wind_comp_angle

# left/right tilt in degrees (rotation about the X axis)
float32 rotX

# forward/backward tilt in degrees (rotation about the Y axis)
float32 rotY

# orientation in degrees (rotation about the Z axis)
float32 rotZ

# estimated altitude (cm)
int32 altd

# linear velocity (mm/sec)
float32 vx

# linear velocity (mm/sec)
float32 vy

# linear velocity (mm/sec)
float32 vz

#linear accelerations (unit: g)
float32 ax
float32 ay
float32 az

#motor commands (unit 0 to 255)
uint8 motor1
uint8 motor2
uint8 motor3
uint8 motor4

#Tags in Vision Detectoion
uint32 tags_count
uint32[] tags_type
uint32[] tags_xc
uint32[] tags_yc
uint32[] tags_width
uint32[] tags_height
float32[] tags_orientation
float32[] tags_distance

#time stamp
float32 tm

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



(provide :ardrone_autonomy/Navdata "e1169f766234363125ac62c9a3f87eeb")


