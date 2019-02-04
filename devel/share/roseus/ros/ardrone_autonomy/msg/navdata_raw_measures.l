;; Auto-generated. Do not edit!


(when (boundp 'ardrone_autonomy::navdata_raw_measures)
  (if (not (find-package "ARDRONE_AUTONOMY"))
    (make-package "ARDRONE_AUTONOMY"))
  (shadow 'navdata_raw_measures (find-package "ARDRONE_AUTONOMY")))
(unless (find-package "ARDRONE_AUTONOMY::NAVDATA_RAW_MEASURES")
  (make-package "ARDRONE_AUTONOMY::NAVDATA_RAW_MEASURES"))

(in-package "ROS")
;;//! \htmlinclude navdata_raw_measures.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ardrone_autonomy::navdata_raw_measures
  :super ros::object
  :slots (_header _drone_time _tag _size _raw_gyros _raw_gyros_110 _vbat_raw _us_debut_echo _us_fin_echo _us_association_echo _us_distance_echo _us_courbe_temps _us_courbe_valeur _us_courbe_ref _flag_echo_ini _nb_echo _sum_echo _alt_temp_raw _gradient ))

(defmethod ardrone_autonomy::navdata_raw_measures
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:drone_time __drone_time) 0.0)
    ((:tag __tag) 0)
    ((:size __size) 0)
    ((:raw_gyros __raw_gyros) (make-array 0 :initial-element 0 :element-type :integer))
    ((:raw_gyros_110 __raw_gyros_110) (make-array 0 :initial-element 0 :element-type :integer))
    ((:vbat_raw __vbat_raw) 0)
    ((:us_debut_echo __us_debut_echo) 0)
    ((:us_fin_echo __us_fin_echo) 0)
    ((:us_association_echo __us_association_echo) 0)
    ((:us_distance_echo __us_distance_echo) 0)
    ((:us_courbe_temps __us_courbe_temps) 0)
    ((:us_courbe_valeur __us_courbe_valeur) 0)
    ((:us_courbe_ref __us_courbe_ref) 0)
    ((:flag_echo_ini __flag_echo_ini) 0)
    ((:nb_echo __nb_echo) 0)
    ((:sum_echo __sum_echo) 0)
    ((:alt_temp_raw __alt_temp_raw) 0)
    ((:gradient __gradient) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _drone_time (float __drone_time))
   (setq _tag (round __tag))
   (setq _size (round __size))
   (setq _raw_gyros __raw_gyros)
   (setq _raw_gyros_110 __raw_gyros_110)
   (setq _vbat_raw (round __vbat_raw))
   (setq _us_debut_echo (round __us_debut_echo))
   (setq _us_fin_echo (round __us_fin_echo))
   (setq _us_association_echo (round __us_association_echo))
   (setq _us_distance_echo (round __us_distance_echo))
   (setq _us_courbe_temps (round __us_courbe_temps))
   (setq _us_courbe_valeur (round __us_courbe_valeur))
   (setq _us_courbe_ref (round __us_courbe_ref))
   (setq _flag_echo_ini (round __flag_echo_ini))
   (setq _nb_echo (round __nb_echo))
   (setq _sum_echo (round __sum_echo))
   (setq _alt_temp_raw (round __alt_temp_raw))
   (setq _gradient (round __gradient))
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
  (:raw_gyros
   (&optional __raw_gyros)
   (if __raw_gyros (setq _raw_gyros __raw_gyros)) _raw_gyros)
  (:raw_gyros_110
   (&optional __raw_gyros_110)
   (if __raw_gyros_110 (setq _raw_gyros_110 __raw_gyros_110)) _raw_gyros_110)
  (:vbat_raw
   (&optional __vbat_raw)
   (if __vbat_raw (setq _vbat_raw __vbat_raw)) _vbat_raw)
  (:us_debut_echo
   (&optional __us_debut_echo)
   (if __us_debut_echo (setq _us_debut_echo __us_debut_echo)) _us_debut_echo)
  (:us_fin_echo
   (&optional __us_fin_echo)
   (if __us_fin_echo (setq _us_fin_echo __us_fin_echo)) _us_fin_echo)
  (:us_association_echo
   (&optional __us_association_echo)
   (if __us_association_echo (setq _us_association_echo __us_association_echo)) _us_association_echo)
  (:us_distance_echo
   (&optional __us_distance_echo)
   (if __us_distance_echo (setq _us_distance_echo __us_distance_echo)) _us_distance_echo)
  (:us_courbe_temps
   (&optional __us_courbe_temps)
   (if __us_courbe_temps (setq _us_courbe_temps __us_courbe_temps)) _us_courbe_temps)
  (:us_courbe_valeur
   (&optional __us_courbe_valeur)
   (if __us_courbe_valeur (setq _us_courbe_valeur __us_courbe_valeur)) _us_courbe_valeur)
  (:us_courbe_ref
   (&optional __us_courbe_ref)
   (if __us_courbe_ref (setq _us_courbe_ref __us_courbe_ref)) _us_courbe_ref)
  (:flag_echo_ini
   (&optional __flag_echo_ini)
   (if __flag_echo_ini (setq _flag_echo_ini __flag_echo_ini)) _flag_echo_ini)
  (:nb_echo
   (&optional __nb_echo)
   (if __nb_echo (setq _nb_echo __nb_echo)) _nb_echo)
  (:sum_echo
   (&optional __sum_echo)
   (if __sum_echo (setq _sum_echo __sum_echo)) _sum_echo)
  (:alt_temp_raw
   (&optional __alt_temp_raw)
   (if __alt_temp_raw (setq _alt_temp_raw __alt_temp_raw)) _alt_temp_raw)
  (:gradient
   (&optional __gradient)
   (if __gradient (setq _gradient __gradient)) _gradient)
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
    ;; int16[] _raw_gyros
    (* 2    (length _raw_gyros)) 4
    ;; int16[] _raw_gyros_110
    (* 2    (length _raw_gyros_110)) 4
    ;; uint32 _vbat_raw
    4
    ;; uint16 _us_debut_echo
    2
    ;; uint16 _us_fin_echo
    2
    ;; uint16 _us_association_echo
    2
    ;; uint16 _us_distance_echo
    2
    ;; uint16 _us_courbe_temps
    2
    ;; uint16 _us_courbe_valeur
    2
    ;; uint16 _us_courbe_ref
    2
    ;; uint16 _flag_echo_ini
    2
    ;; uint16 _nb_echo
    2
    ;; uint32 _sum_echo
    4
    ;; int32 _alt_temp_raw
    4
    ;; int16 _gradient
    2
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
     ;; int16[] _raw_gyros
     (write-long (length _raw_gyros) s)
     (dotimes (i (length _raw_gyros))
       (write-word (elt _raw_gyros i) s)
       )
     ;; int16[] _raw_gyros_110
     (write-long (length _raw_gyros_110) s)
     (dotimes (i (length _raw_gyros_110))
       (write-word (elt _raw_gyros_110 i) s)
       )
     ;; uint32 _vbat_raw
       (write-long _vbat_raw s)
     ;; uint16 _us_debut_echo
       (write-word _us_debut_echo s)
     ;; uint16 _us_fin_echo
       (write-word _us_fin_echo s)
     ;; uint16 _us_association_echo
       (write-word _us_association_echo s)
     ;; uint16 _us_distance_echo
       (write-word _us_distance_echo s)
     ;; uint16 _us_courbe_temps
       (write-word _us_courbe_temps s)
     ;; uint16 _us_courbe_valeur
       (write-word _us_courbe_valeur s)
     ;; uint16 _us_courbe_ref
       (write-word _us_courbe_ref s)
     ;; uint16 _flag_echo_ini
       (write-word _flag_echo_ini s)
     ;; uint16 _nb_echo
       (write-word _nb_echo s)
     ;; uint32 _sum_echo
       (write-long _sum_echo s)
     ;; int32 _alt_temp_raw
       (write-long _alt_temp_raw s)
     ;; int16 _gradient
       (write-word _gradient s)
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
   ;; int16[] _raw_gyros
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _raw_gyros (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _raw_gyros i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;; int16[] _raw_gyros_110
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _raw_gyros_110 (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _raw_gyros_110 i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;; uint32 _vbat_raw
     (setq _vbat_raw (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint16 _us_debut_echo
     (setq _us_debut_echo (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _us_fin_echo
     (setq _us_fin_echo (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _us_association_echo
     (setq _us_association_echo (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _us_distance_echo
     (setq _us_distance_echo (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _us_courbe_temps
     (setq _us_courbe_temps (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _us_courbe_valeur
     (setq _us_courbe_valeur (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _us_courbe_ref
     (setq _us_courbe_ref (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _flag_echo_ini
     (setq _flag_echo_ini (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _nb_echo
     (setq _nb_echo (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint32 _sum_echo
     (setq _sum_echo (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _alt_temp_raw
     (setq _alt_temp_raw (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int16 _gradient
     (setq _gradient (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get ardrone_autonomy::navdata_raw_measures :md5sum-) "4da7145c7478d1eb84be4d5fa4acd9ca")
(setf (get ardrone_autonomy::navdata_raw_measures :datatype-) "ardrone_autonomy/navdata_raw_measures")
(setf (get ardrone_autonomy::navdata_raw_measures :definition-)
      "Header header
float64 drone_time
uint16 tag
uint16 size
int16[] raw_gyros
int16[] raw_gyros_110
uint32 vbat_raw
uint16 us_debut_echo
uint16 us_fin_echo
uint16 us_association_echo
uint16 us_distance_echo
uint16 us_courbe_temps
uint16 us_courbe_valeur
uint16 us_courbe_ref
uint16 flag_echo_ini
uint16 nb_echo
uint32 sum_echo
int32 alt_temp_raw
int16 gradient

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



(provide :ardrone_autonomy/navdata_raw_measures "4da7145c7478d1eb84be4d5fa4acd9ca")


