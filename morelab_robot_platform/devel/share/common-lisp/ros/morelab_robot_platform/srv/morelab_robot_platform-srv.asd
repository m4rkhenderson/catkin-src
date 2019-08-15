
(cl:in-package :asdf)

(defsystem "morelab_robot_platform-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "StopAtDistance" :depends-on ("_package_StopAtDistance"))
    (:file "_package_StopAtDistance" :depends-on ("_package"))
    (:file "DriveWithSpeed" :depends-on ("_package_DriveWithSpeed"))
    (:file "_package_DriveWithSpeed" :depends-on ("_package"))
    (:file "Rotate" :depends-on ("_package_Rotate"))
    (:file "_package_Rotate" :depends-on ("_package"))
    (:file "GetSpeed" :depends-on ("_package_GetSpeed"))
    (:file "_package_GetSpeed" :depends-on ("_package"))
    (:file "GetHeading" :depends-on ("_package_GetHeading"))
    (:file "_package_GetHeading" :depends-on ("_package"))
    (:file "GetStatus" :depends-on ("_package_GetStatus"))
    (:file "_package_GetStatus" :depends-on ("_package"))
    (:file "DriveWithDistance" :depends-on ("_package_DriveWithDistance"))
    (:file "_package_DriveWithDistance" :depends-on ("_package"))
    (:file "ResetEncoder" :depends-on ("_package_ResetEncoder"))
    (:file "_package_ResetEncoder" :depends-on ("_package"))
    (:file "GetDistance" :depends-on ("_package_GetDistance"))
    (:file "_package_GetDistance" :depends-on ("_package"))
    (:file "DriveWithPower" :depends-on ("_package_DriveWithPower"))
    (:file "_package_DriveWithPower" :depends-on ("_package"))
    (:file "Accelerate" :depends-on ("_package_Accelerate"))
    (:file "_package_Accelerate" :depends-on ("_package"))
  ))