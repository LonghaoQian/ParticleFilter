;; Auto-generated. Do not edit!


(when (boundp 'ardrone_autonomy::vector21)
  (if (not (find-package "ARDRONE_AUTONOMY"))
    (make-package "ARDRONE_AUTONOMY"))
  (shadow 'vector21 (find-package "ARDRONE_AUTONOMY")))
(unless (find-package "ARDRONE_AUTONOMY::VECTOR21")
  (make-package "ARDRONE_AUTONOMY::VECTOR21"))

(in-package "ROS")
;;//! \htmlinclude vector21.msg.html


(defclass ardrone_autonomy::vector21
  :super ros::object
  :slots (_x _y ))

(defmethod ardrone_autonomy::vector21
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:serialization-length
   ()
   (+
    ;; float32 _x
    4
    ;; float32 _y
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get ardrone_autonomy::vector21 :md5sum-) "ff8d7d66dd3e4b731ef14a45d38888b6")
(setf (get ardrone_autonomy::vector21 :datatype-) "ardrone_autonomy/vector21")
(setf (get ardrone_autonomy::vector21 :definition-)
      "float32 x
float32 y
")



(provide :ardrone_autonomy/vector21 "ff8d7d66dd3e4b731ef14a45d38888b6")


