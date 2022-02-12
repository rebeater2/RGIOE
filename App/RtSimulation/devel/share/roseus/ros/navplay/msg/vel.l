;; Auto-generated. Do not edit!


(when (boundp 'navplay::vel)
  (if (not (find-package "NAVPLAY"))
    (make-package "NAVPLAY"))
  (shadow 'vel (find-package "NAVPLAY")))
(unless (find-package "NAVPLAY::VEL")
  (make-package "NAVPLAY::VEL"))

(in-package "ROS")
;;//! \htmlinclude vel.msg.html


(defclass navplay::vel
  :super ros::object
  :slots (_gpst _forward _angular ))

(defmethod navplay::vel
  (:init
   (&key
    ((:gpst __gpst) 0.0)
    ((:forward __forward) 0.0)
    ((:angular __angular) 0.0)
    )
   (send-super :init)
   (setq _gpst (float __gpst))
   (setq _forward (float __forward))
   (setq _angular (float __angular))
   self)
  (:gpst
   (&optional __gpst)
   (if __gpst (setq _gpst __gpst)) _gpst)
  (:forward
   (&optional __forward)
   (if __forward (setq _forward __forward)) _forward)
  (:angular
   (&optional __angular)
   (if __angular (setq _angular __angular)) _angular)
  (:serialization-length
   ()
   (+
    ;; float64 _gpst
    8
    ;; float32 _forward
    4
    ;; float32 _angular
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _gpst
       (sys::poke _gpst (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float32 _forward
       (sys::poke _forward (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _angular
       (sys::poke _angular (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _gpst
     (setq _gpst (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float32 _forward
     (setq _forward (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _angular
     (setq _angular (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get navplay::vel :md5sum-) "7e11ee5e5f9a364893511f8eb7b4a576")
(setf (get navplay::vel :datatype-) "navplay/vel")
(setf (get navplay::vel :definition-)
      "float64 gpst
float32 forward
float32 angular
")



(provide :navplay/vel "7e11ee5e5f9a364893511f8eb7b4a576")


