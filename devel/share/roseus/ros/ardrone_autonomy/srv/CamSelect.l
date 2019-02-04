;; Auto-generated. Do not edit!


(when (boundp 'ardrone_autonomy::CamSelect)
  (if (not (find-package "ARDRONE_AUTONOMY"))
    (make-package "ARDRONE_AUTONOMY"))
  (shadow 'CamSelect (find-package "ARDRONE_AUTONOMY")))
(unless (find-package "ARDRONE_AUTONOMY::CAMSELECT")
  (make-package "ARDRONE_AUTONOMY::CAMSELECT"))
(unless (find-package "ARDRONE_AUTONOMY::CAMSELECTREQUEST")
  (make-package "ARDRONE_AUTONOMY::CAMSELECTREQUEST"))
(unless (find-package "ARDRONE_AUTONOMY::CAMSELECTRESPONSE")
  (make-package "ARDRONE_AUTONOMY::CAMSELECTRESPONSE"))

(in-package "ROS")





(defclass ardrone_autonomy::CamSelectRequest
  :super ros::object
  :slots (_channel ))

(defmethod ardrone_autonomy::CamSelectRequest
  (:init
   (&key
    ((:channel __channel) 0)
    )
   (send-super :init)
   (setq _channel (round __channel))
   self)
  (:channel
   (&optional __channel)
   (if __channel (setq _channel __channel)) _channel)
  (:serialization-length
   ()
   (+
    ;; uint8 _channel
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _channel
       (write-byte _channel s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _channel
     (setq _channel (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(defclass ardrone_autonomy::CamSelectResponse
  :super ros::object
  :slots (_result ))

(defmethod ardrone_autonomy::CamSelectResponse
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

(defclass ardrone_autonomy::CamSelect
  :super ros::object
  :slots ())

(setf (get ardrone_autonomy::CamSelect :md5sum-) "bbeb5212f8ee1d6da7ff0d1169124280")
(setf (get ardrone_autonomy::CamSelect :datatype-) "ardrone_autonomy/CamSelect")
(setf (get ardrone_autonomy::CamSelect :request) ardrone_autonomy::CamSelectRequest)
(setf (get ardrone_autonomy::CamSelect :response) ardrone_autonomy::CamSelectResponse)

(defmethod ardrone_autonomy::CamSelectRequest
  (:response () (instance ardrone_autonomy::CamSelectResponse :init)))

(setf (get ardrone_autonomy::CamSelectRequest :md5sum-) "bbeb5212f8ee1d6da7ff0d1169124280")
(setf (get ardrone_autonomy::CamSelectRequest :datatype-) "ardrone_autonomy/CamSelectRequest")
(setf (get ardrone_autonomy::CamSelectRequest :definition-)
      "uint8 channel
---
bool result

")

(setf (get ardrone_autonomy::CamSelectResponse :md5sum-) "bbeb5212f8ee1d6da7ff0d1169124280")
(setf (get ardrone_autonomy::CamSelectResponse :datatype-) "ardrone_autonomy/CamSelectResponse")
(setf (get ardrone_autonomy::CamSelectResponse :definition-)
      "uint8 channel
---
bool result

")



(provide :ardrone_autonomy/CamSelect "bbeb5212f8ee1d6da7ff0d1169124280")


