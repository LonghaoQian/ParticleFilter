;; Auto-generated. Do not edit!


(when (boundp 'ardrone_autonomy::matrix33)
  (if (not (find-package "ARDRONE_AUTONOMY"))
    (make-package "ARDRONE_AUTONOMY"))
  (shadow 'matrix33 (find-package "ARDRONE_AUTONOMY")))
(unless (find-package "ARDRONE_AUTONOMY::MATRIX33")
  (make-package "ARDRONE_AUTONOMY::MATRIX33"))

(in-package "ROS")
;;//! \htmlinclude matrix33.msg.html


(defclass ardrone_autonomy::matrix33
  :super ros::object
  :slots (_m11 _m12 _m13 _m21 _m22 _m23 _m31 _m32 _m33 ))

(defmethod ardrone_autonomy::matrix33
  (:init
   (&key
    ((:m11 __m11) 0.0)
    ((:m12 __m12) 0.0)
    ((:m13 __m13) 0.0)
    ((:m21 __m21) 0.0)
    ((:m22 __m22) 0.0)
    ((:m23 __m23) 0.0)
    ((:m31 __m31) 0.0)
    ((:m32 __m32) 0.0)
    ((:m33 __m33) 0.0)
    )
   (send-super :init)
   (setq _m11 (float __m11))
   (setq _m12 (float __m12))
   (setq _m13 (float __m13))
   (setq _m21 (float __m21))
   (setq _m22 (float __m22))
   (setq _m23 (float __m23))
   (setq _m31 (float __m31))
   (setq _m32 (float __m32))
   (setq _m33 (float __m33))
   self)
  (:m11
   (&optional __m11)
   (if __m11 (setq _m11 __m11)) _m11)
  (:m12
   (&optional __m12)
   (if __m12 (setq _m12 __m12)) _m12)
  (:m13
   (&optional __m13)
   (if __m13 (setq _m13 __m13)) _m13)
  (:m21
   (&optional __m21)
   (if __m21 (setq _m21 __m21)) _m21)
  (:m22
   (&optional __m22)
   (if __m22 (setq _m22 __m22)) _m22)
  (:m23
   (&optional __m23)
   (if __m23 (setq _m23 __m23)) _m23)
  (:m31
   (&optional __m31)
   (if __m31 (setq _m31 __m31)) _m31)
  (:m32
   (&optional __m32)
   (if __m32 (setq _m32 __m32)) _m32)
  (:m33
   (&optional __m33)
   (if __m33 (setq _m33 __m33)) _m33)
  (:serialization-length
   ()
   (+
    ;; float32 _m11
    4
    ;; float32 _m12
    4
    ;; float32 _m13
    4
    ;; float32 _m21
    4
    ;; float32 _m22
    4
    ;; float32 _m23
    4
    ;; float32 _m31
    4
    ;; float32 _m32
    4
    ;; float32 _m33
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _m11
       (sys::poke _m11 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _m12
       (sys::poke _m12 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _m13
       (sys::poke _m13 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _m21
       (sys::poke _m21 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _m22
       (sys::poke _m22 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _m23
       (sys::poke _m23 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _m31
       (sys::poke _m31 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _m32
       (sys::poke _m32 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _m33
       (sys::poke _m33 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _m11
     (setq _m11 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _m12
     (setq _m12 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _m13
     (setq _m13 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _m21
     (setq _m21 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _m22
     (setq _m22 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _m23
     (setq _m23 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _m31
     (setq _m31 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _m32
     (setq _m32 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _m33
     (setq _m33 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get ardrone_autonomy::matrix33 :md5sum-) "d6abfd8481fcb7c359a8d5d9388fa257")
(setf (get ardrone_autonomy::matrix33 :datatype-) "ardrone_autonomy/matrix33")
(setf (get ardrone_autonomy::matrix33 :definition-)
      "float32 m11
float32 m12
float32 m13
float32 m21
float32 m22
float32 m23
float32 m31
float32 m32
float32 m33
")



(provide :ardrone_autonomy/matrix33 "d6abfd8481fcb7c359a8d5d9388fa257")


