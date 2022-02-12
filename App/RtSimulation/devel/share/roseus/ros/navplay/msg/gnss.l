;; Auto-generated. Do not edit!


(when (boundp 'navplay::gnss)
  (if (not (find-package "NAVPLAY"))
    (make-package "NAVPLAY"))
  (shadow 'gnss (find-package "NAVPLAY")))
(unless (find-package "NAVPLAY::GNSS")
  (make-package "NAVPLAY::GNSS"))

(in-package "ROS")
;;//! \htmlinclude gnss.msg.html


(defclass navplay::gnss
  :super ros::object
  :slots (_gpst _lat _lon _h _pos_std _mode _ns ))

(defmethod navplay::gnss
  (:init
   (&key
    ((:gpst __gpst) 0.0)
    ((:lat __lat) 0.0)
    ((:lon __lon) 0.0)
    ((:h __h) 0.0)
    ((:pos_std __pos_std) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:mode __mode) 0)
    ((:ns __ns) 0)
    )
   (send-super :init)
   (setq _gpst (float __gpst))
   (setq _lat (float __lat))
   (setq _lon (float __lon))
   (setq _h (float __h))
   (setq _pos_std __pos_std)
   (setq _mode (round __mode))
   (setq _ns (round __ns))
   self)
  (:gpst
   (&optional __gpst)
   (if __gpst (setq _gpst __gpst)) _gpst)
  (:lat
   (&optional __lat)
   (if __lat (setq _lat __lat)) _lat)
  (:lon
   (&optional __lon)
   (if __lon (setq _lon __lon)) _lon)
  (:h
   (&optional __h)
   (if __h (setq _h __h)) _h)
  (:pos_std
   (&optional __pos_std)
   (if __pos_std (setq _pos_std __pos_std)) _pos_std)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:ns
   (&optional __ns)
   (if __ns (setq _ns __ns)) _ns)
  (:serialization-length
   ()
   (+
    ;; float64 _gpst
    8
    ;; float64 _lat
    8
    ;; float64 _lon
    8
    ;; float32 _h
    4
    ;; float32[3] _pos_std
    (* 4    3)
    ;; uint8 _mode
    1
    ;; uint8 _ns
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _gpst
       (sys::poke _gpst (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _lat
       (sys::poke _lat (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _lon
       (sys::poke _lon (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float32 _h
       (sys::poke _h (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32[3] _pos_std
     (dotimes (i 3)
       (sys::poke (elt _pos_std i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; uint8 _mode
       (write-byte _mode s)
     ;; uint8 _ns
       (write-byte _ns s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _gpst
     (setq _gpst (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _lat
     (setq _lat (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _lon
     (setq _lon (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float32 _h
     (setq _h (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32[3] _pos_std
   (dotimes (i (length _pos_std))
     (setf (elt _pos_std i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; uint8 _mode
     (setq _mode (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _ns
     (setq _ns (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get navplay::gnss :md5sum-) "d8b3bc143a1901908ef5f7a6d985bd0b")
(setf (get navplay::gnss :datatype-) "navplay/gnss")
(setf (get navplay::gnss :definition-)
      "float64 gpst
float64 lat
float64 lon
float32 h
float32[3] pos_std
uint8 mode
uint8 ns
")



(provide :navplay/gnss "d8b3bc143a1901908ef5f7a6d985bd0b")


