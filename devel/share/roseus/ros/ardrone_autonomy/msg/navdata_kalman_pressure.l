;; Auto-generated. Do not edit!


(when (boundp 'ardrone_autonomy::navdata_kalman_pressure)
  (if (not (find-package "ARDRONE_AUTONOMY"))
    (make-package "ARDRONE_AUTONOMY"))
  (shadow 'navdata_kalman_pressure (find-package "ARDRONE_AUTONOMY")))
(unless (find-package "ARDRONE_AUTONOMY::NAVDATA_KALMAN_PRESSURE")
  (make-package "ARDRONE_AUTONOMY::NAVDATA_KALMAN_PRESSURE"))

(in-package "ROS")
;;//! \htmlinclude navdata_kalman_pressure.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ardrone_autonomy::navdata_kalman_pressure
  :super ros::object
  :slots (_header _drone_time _tag _size _offset_pressure _est_z _est_zdot _est_bias_PWM _est_biais_pression _offset_US _prediction_US _cov_alt _cov_PWM _cov_vitesse _bool_effet_sol _somme_inno _flag_rejet_US _u_multisinus _gaz_altitude _Flag_multisinus _Flag_multisinus_debut ))

(defmethod ardrone_autonomy::navdata_kalman_pressure
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:drone_time __drone_time) 0.0)
    ((:tag __tag) 0)
    ((:size __size) 0)
    ((:offset_pressure __offset_pressure) 0.0)
    ((:est_z __est_z) 0.0)
    ((:est_zdot __est_zdot) 0.0)
    ((:est_bias_PWM __est_bias_PWM) 0.0)
    ((:est_biais_pression __est_biais_pression) 0.0)
    ((:offset_US __offset_US) 0.0)
    ((:prediction_US __prediction_US) 0.0)
    ((:cov_alt __cov_alt) 0.0)
    ((:cov_PWM __cov_PWM) 0.0)
    ((:cov_vitesse __cov_vitesse) 0.0)
    ((:bool_effet_sol __bool_effet_sol) 0)
    ((:somme_inno __somme_inno) 0.0)
    ((:flag_rejet_US __flag_rejet_US) 0)
    ((:u_multisinus __u_multisinus) 0.0)
    ((:gaz_altitude __gaz_altitude) 0.0)
    ((:Flag_multisinus __Flag_multisinus) 0)
    ((:Flag_multisinus_debut __Flag_multisinus_debut) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _drone_time (float __drone_time))
   (setq _tag (round __tag))
   (setq _size (round __size))
   (setq _offset_pressure (float __offset_pressure))
   (setq _est_z (float __est_z))
   (setq _est_zdot (float __est_zdot))
   (setq _est_bias_PWM (float __est_bias_PWM))
   (setq _est_biais_pression (float __est_biais_pression))
   (setq _offset_US (float __offset_US))
   (setq _prediction_US (float __prediction_US))
   (setq _cov_alt (float __cov_alt))
   (setq _cov_PWM (float __cov_PWM))
   (setq _cov_vitesse (float __cov_vitesse))
   (setq _bool_effet_sol (round __bool_effet_sol))
   (setq _somme_inno (float __somme_inno))
   (setq _flag_rejet_US (round __flag_rejet_US))
   (setq _u_multisinus (float __u_multisinus))
   (setq _gaz_altitude (float __gaz_altitude))
   (setq _Flag_multisinus (round __Flag_multisinus))
   (setq _Flag_multisinus_debut (round __Flag_multisinus_debut))
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
  (:offset_pressure
   (&optional __offset_pressure)
   (if __offset_pressure (setq _offset_pressure __offset_pressure)) _offset_pressure)
  (:est_z
   (&optional __est_z)
   (if __est_z (setq _est_z __est_z)) _est_z)
  (:est_zdot
   (&optional __est_zdot)
   (if __est_zdot (setq _est_zdot __est_zdot)) _est_zdot)
  (:est_bias_PWM
   (&optional __est_bias_PWM)
   (if __est_bias_PWM (setq _est_bias_PWM __est_bias_PWM)) _est_bias_PWM)
  (:est_biais_pression
   (&optional __est_biais_pression)
   (if __est_biais_pression (setq _est_biais_pression __est_biais_pression)) _est_biais_pression)
  (:offset_US
   (&optional __offset_US)
   (if __offset_US (setq _offset_US __offset_US)) _offset_US)
  (:prediction_US
   (&optional __prediction_US)
   (if __prediction_US (setq _prediction_US __prediction_US)) _prediction_US)
  (:cov_alt
   (&optional __cov_alt)
   (if __cov_alt (setq _cov_alt __cov_alt)) _cov_alt)
  (:cov_PWM
   (&optional __cov_PWM)
   (if __cov_PWM (setq _cov_PWM __cov_PWM)) _cov_PWM)
  (:cov_vitesse
   (&optional __cov_vitesse)
   (if __cov_vitesse (setq _cov_vitesse __cov_vitesse)) _cov_vitesse)
  (:bool_effet_sol
   (&optional __bool_effet_sol)
   (if __bool_effet_sol (setq _bool_effet_sol __bool_effet_sol)) _bool_effet_sol)
  (:somme_inno
   (&optional __somme_inno)
   (if __somme_inno (setq _somme_inno __somme_inno)) _somme_inno)
  (:flag_rejet_US
   (&optional __flag_rejet_US)
   (if __flag_rejet_US (setq _flag_rejet_US __flag_rejet_US)) _flag_rejet_US)
  (:u_multisinus
   (&optional __u_multisinus)
   (if __u_multisinus (setq _u_multisinus __u_multisinus)) _u_multisinus)
  (:gaz_altitude
   (&optional __gaz_altitude)
   (if __gaz_altitude (setq _gaz_altitude __gaz_altitude)) _gaz_altitude)
  (:Flag_multisinus
   (&optional __Flag_multisinus)
   (if __Flag_multisinus (setq _Flag_multisinus __Flag_multisinus)) _Flag_multisinus)
  (:Flag_multisinus_debut
   (&optional __Flag_multisinus_debut)
   (if __Flag_multisinus_debut (setq _Flag_multisinus_debut __Flag_multisinus_debut)) _Flag_multisinus_debut)
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
    ;; float32 _offset_pressure
    4
    ;; float32 _est_z
    4
    ;; float32 _est_zdot
    4
    ;; float32 _est_bias_PWM
    4
    ;; float32 _est_biais_pression
    4
    ;; float32 _offset_US
    4
    ;; float32 _prediction_US
    4
    ;; float32 _cov_alt
    4
    ;; float32 _cov_PWM
    4
    ;; float32 _cov_vitesse
    4
    ;; int32 _bool_effet_sol
    4
    ;; float32 _somme_inno
    4
    ;; int32 _flag_rejet_US
    4
    ;; float32 _u_multisinus
    4
    ;; float32 _gaz_altitude
    4
    ;; int32 _Flag_multisinus
    4
    ;; int32 _Flag_multisinus_debut
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
     ;; float32 _offset_pressure
       (sys::poke _offset_pressure (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _est_z
       (sys::poke _est_z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _est_zdot
       (sys::poke _est_zdot (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _est_bias_PWM
       (sys::poke _est_bias_PWM (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _est_biais_pression
       (sys::poke _est_biais_pression (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _offset_US
       (sys::poke _offset_US (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _prediction_US
       (sys::poke _prediction_US (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _cov_alt
       (sys::poke _cov_alt (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _cov_PWM
       (sys::poke _cov_PWM (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _cov_vitesse
       (sys::poke _cov_vitesse (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32 _bool_effet_sol
       (write-long _bool_effet_sol s)
     ;; float32 _somme_inno
       (sys::poke _somme_inno (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32 _flag_rejet_US
       (write-long _flag_rejet_US s)
     ;; float32 _u_multisinus
       (sys::poke _u_multisinus (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _gaz_altitude
       (sys::poke _gaz_altitude (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32 _Flag_multisinus
       (write-long _Flag_multisinus s)
     ;; int32 _Flag_multisinus_debut
       (write-long _Flag_multisinus_debut s)
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
   ;; float32 _offset_pressure
     (setq _offset_pressure (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _est_z
     (setq _est_z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _est_zdot
     (setq _est_zdot (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _est_bias_PWM
     (setq _est_bias_PWM (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _est_biais_pression
     (setq _est_biais_pression (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _offset_US
     (setq _offset_US (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _prediction_US
     (setq _prediction_US (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _cov_alt
     (setq _cov_alt (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _cov_PWM
     (setq _cov_PWM (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _cov_vitesse
     (setq _cov_vitesse (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32 _bool_effet_sol
     (setq _bool_effet_sol (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _somme_inno
     (setq _somme_inno (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32 _flag_rejet_US
     (setq _flag_rejet_US (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _u_multisinus
     (setq _u_multisinus (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _gaz_altitude
     (setq _gaz_altitude (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32 _Flag_multisinus
     (setq _Flag_multisinus (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _Flag_multisinus_debut
     (setq _Flag_multisinus_debut (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get ardrone_autonomy::navdata_kalman_pressure :md5sum-) "70734b6caff0fb7ea6fc88ffeea5cde5")
(setf (get ardrone_autonomy::navdata_kalman_pressure :datatype-) "ardrone_autonomy/navdata_kalman_pressure")
(setf (get ardrone_autonomy::navdata_kalman_pressure :definition-)
      "Header header
float64 drone_time
uint16 tag
uint16 size
float32 offset_pressure
float32 est_z
float32 est_zdot
float32 est_bias_PWM
float32 est_biais_pression
float32 offset_US
float32 prediction_US
float32 cov_alt
float32 cov_PWM
float32 cov_vitesse
int32 bool_effet_sol
float32 somme_inno
int32 flag_rejet_US
float32 u_multisinus
float32 gaz_altitude
int32 Flag_multisinus
int32 Flag_multisinus_debut

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



(provide :ardrone_autonomy/navdata_kalman_pressure "70734b6caff0fb7ea6fc88ffeea5cde5")


