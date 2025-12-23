;; Auto-generated. Do not edit!


(when (boundp 'scout_msgs::ScoutRsStatus)
  (if (not (find-package "SCOUT_MSGS"))
    (make-package "SCOUT_MSGS"))
  (shadow 'ScoutRsStatus (find-package "SCOUT_MSGS")))
(unless (find-package "SCOUT_MSGS::SCOUTRSSTATUS")
  (make-package "SCOUT_MSGS::SCOUTRSSTATUS"))

(in-package "ROS")
;;//! \htmlinclude ScoutRsStatus.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass scout_msgs::ScoutRsStatus
  :super ros::object
  :slots (_header _stick_left_h _stick_left_v _stick_right_h _stick_right_v _swa _swb _swc _swd _var_a ))

(defmethod scout_msgs::ScoutRsStatus
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:stick_left_h __stick_left_h) 0)
    ((:stick_left_v __stick_left_v) 0)
    ((:stick_right_h __stick_right_h) 0)
    ((:stick_right_v __stick_right_v) 0)
    ((:swa __swa) 0)
    ((:swb __swb) 0)
    ((:swc __swc) 0)
    ((:swd __swd) 0)
    ((:var_a __var_a) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _stick_left_h (round __stick_left_h))
   (setq _stick_left_v (round __stick_left_v))
   (setq _stick_right_h (round __stick_right_h))
   (setq _stick_right_v (round __stick_right_v))
   (setq _swa (round __swa))
   (setq _swb (round __swb))
   (setq _swc (round __swc))
   (setq _swd (round __swd))
   (setq _var_a (round __var_a))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:stick_left_h
   (&optional __stick_left_h)
   (if __stick_left_h (setq _stick_left_h __stick_left_h)) _stick_left_h)
  (:stick_left_v
   (&optional __stick_left_v)
   (if __stick_left_v (setq _stick_left_v __stick_left_v)) _stick_left_v)
  (:stick_right_h
   (&optional __stick_right_h)
   (if __stick_right_h (setq _stick_right_h __stick_right_h)) _stick_right_h)
  (:stick_right_v
   (&optional __stick_right_v)
   (if __stick_right_v (setq _stick_right_v __stick_right_v)) _stick_right_v)
  (:swa
   (&optional __swa)
   (if __swa (setq _swa __swa)) _swa)
  (:swb
   (&optional __swb)
   (if __swb (setq _swb __swb)) _swb)
  (:swc
   (&optional __swc)
   (if __swc (setq _swc __swc)) _swc)
  (:swd
   (&optional __swd)
   (if __swd (setq _swd __swd)) _swd)
  (:var_a
   (&optional __var_a)
   (if __var_a (setq _var_a __var_a)) _var_a)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int8 _stick_left_h
    1
    ;; int8 _stick_left_v
    1
    ;; int8 _stick_right_h
    1
    ;; int8 _stick_right_v
    1
    ;; uint8 _swa
    1
    ;; uint8 _swb
    1
    ;; uint8 _swc
    1
    ;; uint8 _swd
    1
    ;; uint8 _var_a
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int8 _stick_left_h
       (write-byte _stick_left_h s)
     ;; int8 _stick_left_v
       (write-byte _stick_left_v s)
     ;; int8 _stick_right_h
       (write-byte _stick_right_h s)
     ;; int8 _stick_right_v
       (write-byte _stick_right_v s)
     ;; uint8 _swa
       (write-byte _swa s)
     ;; uint8 _swb
       (write-byte _swb s)
     ;; uint8 _swc
       (write-byte _swc s)
     ;; uint8 _swd
       (write-byte _swd s)
     ;; uint8 _var_a
       (write-byte _var_a s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int8 _stick_left_h
     (setq _stick_left_h (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _stick_left_h 127) (setq _stick_left_h (- _stick_left_h 256)))
   ;; int8 _stick_left_v
     (setq _stick_left_v (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _stick_left_v 127) (setq _stick_left_v (- _stick_left_v 256)))
   ;; int8 _stick_right_h
     (setq _stick_right_h (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _stick_right_h 127) (setq _stick_right_h (- _stick_right_h 256)))
   ;; int8 _stick_right_v
     (setq _stick_right_v (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _stick_right_v 127) (setq _stick_right_v (- _stick_right_v 256)))
   ;; uint8 _swa
     (setq _swa (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _swb
     (setq _swb (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _swc
     (setq _swc (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _swd
     (setq _swd (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _var_a
     (setq _var_a (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get scout_msgs::ScoutRsStatus :md5sum-) "e023f9f94b48268948746d41e8f52039")
(setf (get scout_msgs::ScoutRsStatus :datatype-) "scout_msgs/ScoutRsStatus")
(setf (get scout_msgs::ScoutRsStatus :definition-)
      "Header header

int8 stick_left_h
int8 stick_left_v
int8 stick_right_h
int8 stick_right_v

uint8 swa
uint8 swb
uint8 swc
uint8 swd

uint8 var_a

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
string frame_id

")



(provide :scout_msgs/ScoutRsStatus "e023f9f94b48268948746d41e8f52039")


