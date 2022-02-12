;; Auto-generated. Do not edit!


(when (boundp 'navplay::imu)
  (if (not (find-package "NAVPLAY"))
    (make-package "NAVPLAY"))
  (shadow 'imu (find-package "NAVPLAY")))
(unless (find-package "NAVPLAY::IMU")
  (make-package "NAVPLAY::IMU"))

(in-package "ROS")
;;//! \htmlinclude imu.msg.html


(defclass navplay::imu
  :super ros::object
  :slots (_gpst _acce _gyro ))

(defmethod navplay::imu
  (:init
   (&key
    ((:gpst __gpst) 0.0)
    ((:acce __acce) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:gyro __gyro) (make-array 3 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _gpst (float __gpst))
   (setq _acce __acce)
   (setq _gyro __gyro)
   self)
  (:gpst
   (&optional __gpst)
   (if __gpst (setq _gpst __gpst)) _gpst)
  (:acce
   (&optional __acce)
   (if __acce (setq _acce __acce)) _acce)
  (:gyro
   (&optional __gyro)
   (if __gyro (setq _gyro __gyro)) _gyro)
  (:serialization-length
   ()
   (+
    ;; float64 _gpst
    8
    ;; float64[3] _acce
    (* 8    3)
    ;; float64[3] _gyro
    (* 8    3)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _gpst
       (sys::poke _gpst (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64[3] _acce
     (dotimes (i 3)
       (sys::poke (elt _acce i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[3] _gyro
     (dotimes (i 3)
       (sys::poke (elt _gyro i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _gpst
     (setq _gpst (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64[3] _acce
   (dotimes (i (length _acce))
     (setf (elt _acce i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[3] _gyro
   (dotimes (i (length _gyro))
     (setf (elt _gyro i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;;
   self)
  )

(setf (get navplay::imu :md5sum-) "08a3930f07de2d670d1b2b4f3b2dd41c")
(setf (get navplay::imu :datatype-) "navplay/imu")
(setf (get navplay::imu :definition-)
      "float64 gpst
float64[3] acce
float64[3] gyro
")



(provide :navplay/imu "08a3930f07de2d670d1b2b4f3b2dd41c")


