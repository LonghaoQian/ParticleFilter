;; Auto-generated. Do not edit!


(when (boundp 'keyboard_reader::Key)
  (if (not (find-package "KEYBOARD_READER"))
    (make-package "KEYBOARD_READER"))
  (shadow 'Key (find-package "KEYBOARD_READER")))
(unless (find-package "KEYBOARD_READER::KEY")
  (make-package "KEYBOARD_READER::KEY"))

(in-package "ROS")
;;//! \htmlinclude Key.msg.html


(defclass keyboard_reader::Key
  :super ros::object
  :slots (_key_code _key_name _key_pressed ))

(defmethod keyboard_reader::Key
  (:init
   (&key
    ((:key_code __key_code) 0)
    ((:key_name __key_name) "")
    ((:key_pressed __key_pressed) nil)
    )
   (send-super :init)
   (setq _key_code (round __key_code))
   (setq _key_name (string __key_name))
   (setq _key_pressed __key_pressed)
   self)
  (:key_code
   (&optional __key_code)
   (if __key_code (setq _key_code __key_code)) _key_code)
  (:key_name
   (&optional __key_name)
   (if __key_name (setq _key_name __key_name)) _key_name)
  (:key_pressed
   (&optional __key_pressed)
   (if __key_pressed (setq _key_pressed __key_pressed)) _key_pressed)
  (:serialization-length
   ()
   (+
    ;; uint16 _key_code
    2
    ;; string _key_name
    4 (length _key_name)
    ;; bool _key_pressed
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint16 _key_code
       (write-word _key_code s)
     ;; string _key_name
       (write-long (length _key_name) s) (princ _key_name s)
     ;; bool _key_pressed
       (if _key_pressed (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint16 _key_code
     (setq _key_code (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; string _key_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _key_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _key_pressed
     (setq _key_pressed (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get keyboard_reader::Key :md5sum-) "9709d7232efeba3860fec95e77ac1ae6")
(setf (get keyboard_reader::Key :datatype-) "keyboard_reader/Key")
(setf (get keyboard_reader::Key :definition-)
      "# Key code as defined in linux/inupt.h
uint16 key_code

# Key name string as defined in evtest, see http://elinux.org/images/9/93/Evtest.c
string key_name

# 'True' if key was pressed, 'False' otherwise
bool key_pressed
")



(provide :keyboard_reader/Key "9709d7232efeba3860fec95e77ac1ae6")


