
(cl:in-package :asdf)

(defsystem "navplay-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "gnss" :depends-on ("_package_gnss"))
    (:file "_package_gnss" :depends-on ("_package"))
    (:file "imu" :depends-on ("_package_imu"))
    (:file "_package_imu" :depends-on ("_package"))
    (:file "pva" :depends-on ("_package_pva"))
    (:file "_package_pva" :depends-on ("_package"))
    (:file "vel" :depends-on ("_package_vel"))
    (:file "_package_vel" :depends-on ("_package"))
  ))