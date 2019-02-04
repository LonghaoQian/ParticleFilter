;; Auto-generated. Do not edit!


(when (boundp 'ardrone_autonomy::LedAnim)
  (if (not (find-package "ARDRONE_AUTONOMY"))
    (make-package "ARDRONE_AUTONOMY"))
  (shadow 'LedAnim (find-package "ARDRONE_AUTONOMY")))
(unless (find-package "ARDRONE_AUTONOMY::LEDANIM")
  (make-package "ARDRONE_AUTONOMY::LEDANIM"))
(unless (find-package "ARDRONE_AUTONOMY::LEDANIMREQUEST")
  (make-package "ARDRONE_AUTONOMY::LEDANIMREQUEST"))
(unless (find-package "ARDRONE_AUTONOMY::LEDANIMRESPONSE")
  (make-package "ARDRONE_AUTONOMY::LEDANIMRESPONSE"))

(in-package "ROS")





(defclass ardrone_autonomy::LedAnimRequest
  :super ros::object
  :slots (_type _freq _duration ))

(defmethod ardrone_autonomy::LedAnimRequest
  (:init
   (&key
    ((:type __type) 0)
    ((:freq __freq) 0.0)
    ((:duration __duration) 0)
    )
   (send-super :init)
   (setq _type (round __type))
   (setq _freq (float __freq))
   (setq _duration (round __duration))
   self)
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:freq
   (&optional __freq)
   (if __freq (setq _freq __freq)) _freq)
  (:duration
   (&optional __duration)
   (if __duration (setq _duration __duration)) _duration)
  (:serialization-length
   ()
   (+
    ;; uint8 _type
    1
    ;; float32 _freq
    4
    ;; uint8 _duration
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _type
       (write-byte _type s)
     ;; float32 _freq
       (sys::poke _freq (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint8 _duration
       (write-byte _duration s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _type
     (setq _type (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _freq
     (setq _freq (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint8 _duration
     (setq _duration (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(defclass ardrone_autonomy::LedAnimResponse
  :super ros::object
  :slots (_result ))

(defmethod ardrone_autonomy::LedAnimResponse
  (:init
   (&key
    ((:result __result) nil)
    )
   (send-super :init)
   (setq _result __result)
   self)
  (:result
   (&optional __result)
   (if __result (setq _result __result)) _result)
  (:serialization-length
   ()
   (+
    ;; bool _result
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _result
       (if _result (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _result
     (setq _result (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass ardrone_autonomy::LedAnim
  :super ros::object
  :slots ())

(setf (get ardrone_autonomy::LedAnim :md5sum-) "0447d1620f8ba70a5b1fc2d89e406549")
(setf (get ardrone_autonomy::LedAnim :datatype-) "ardrone_autonomy/LedAnim")
(setf (get ardrone_autonomy::LedAnim :request) ardrone_autonomy::LedAnimRequest)
(setf (get ardrone_autonomy::LedAnim :response) ardrone_autonomy::LedAnimResponse)

(defmethod ardrone_autonomy::LedAnimRequest
  (:response () (instance ardrone_autonomy::LedAnimResponse :init)))

(setf (get ardrone_autonomy::LedAnimRequest :md5sum-) "0447d1620f8ba70a5b1fc2d89e406549")
(setf (get ardrone_autonomy::LedAnimRequest :datatype-) "ardrone_autonomy/LedAnimRequest")
(setf (get ardrone_autonomy::LedAnimRequest :definition-)
      "













uint8 type


float32 freq


uint8 duration

---
bool result

")

(setf (get ardrone_autonomy::LedAnimResponse :md5sum-) "0447d1620f8ba70a5b1fc2d89e406549")
(setf (get ardrone_autonomy::LedAnimResponse :datatype-) "ardrone_autonomy/LedAnimResponse")
(setf (get ardrone_autonomy::LedAnimResponse :definition-)
      "













uint8 type


float32 freq


uint8 duration

---
bool result

")



(provide :ardrone_autonomy/LedAnim "0447d1620f8ba70a5b1fc2d89e406549")


