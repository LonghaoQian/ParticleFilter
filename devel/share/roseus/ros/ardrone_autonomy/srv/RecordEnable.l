;; Auto-generated. Do not edit!


(when (boundp 'ardrone_autonomy::RecordEnable)
  (if (not (find-package "ARDRONE_AUTONOMY"))
    (make-package "ARDRONE_AUTONOMY"))
  (shadow 'RecordEnable (find-package "ARDRONE_AUTONOMY")))
(unless (find-package "ARDRONE_AUTONOMY::RECORDENABLE")
  (make-package "ARDRONE_AUTONOMY::RECORDENABLE"))
(unless (find-package "ARDRONE_AUTONOMY::RECORDENABLEREQUEST")
  (make-package "ARDRONE_AUTONOMY::RECORDENABLEREQUEST"))
(unless (find-package "ARDRONE_AUTONOMY::RECORDENABLERESPONSE")
  (make-package "ARDRONE_AUTONOMY::RECORDENABLERESPONSE"))

(in-package "ROS")





(defclass ardrone_autonomy::RecordEnableRequest
  :super ros::object
  :slots (_enable ))

(defmethod ardrone_autonomy::RecordEnableRequest
  (:init
   (&key
    ((:enable __enable) nil)
    )
   (send-super :init)
   (setq _enable __enable)
   self)
  (:enable
   (&optional __enable)
   (if __enable (setq _enable __enable)) _enable)
  (:serialization-length
   ()
   (+
    ;; bool _enable
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _enable
       (if _enable (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _enable
     (setq _enable (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass ardrone_autonomy::RecordEnableResponse
  :super ros::object
  :slots (_result ))

(defmethod ardrone_autonomy::RecordEnableResponse
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

(defclass ardrone_autonomy::RecordEnable
  :super ros::object
  :slots ())

(setf (get ardrone_autonomy::RecordEnable :md5sum-) "29d58f387352c15c4e4f5763022ae875")
(setf (get ardrone_autonomy::RecordEnable :datatype-) "ardrone_autonomy/RecordEnable")
(setf (get ardrone_autonomy::RecordEnable :request) ardrone_autonomy::RecordEnableRequest)
(setf (get ardrone_autonomy::RecordEnable :response) ardrone_autonomy::RecordEnableResponse)

(defmethod ardrone_autonomy::RecordEnableRequest
  (:response () (instance ardrone_autonomy::RecordEnableResponse :init)))

(setf (get ardrone_autonomy::RecordEnableRequest :md5sum-) "29d58f387352c15c4e4f5763022ae875")
(setf (get ardrone_autonomy::RecordEnableRequest :datatype-) "ardrone_autonomy/RecordEnableRequest")
(setf (get ardrone_autonomy::RecordEnableRequest :definition-)
      "bool enable
---
bool result
")

(setf (get ardrone_autonomy::RecordEnableResponse :md5sum-) "29d58f387352c15c4e4f5763022ae875")
(setf (get ardrone_autonomy::RecordEnableResponse :datatype-) "ardrone_autonomy/RecordEnableResponse")
(setf (get ardrone_autonomy::RecordEnableResponse :definition-)
      "bool enable
---
bool result
")



(provide :ardrone_autonomy/RecordEnable "29d58f387352c15c4e4f5763022ae875")


