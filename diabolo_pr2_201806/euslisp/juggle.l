(ros::load-ros-manifest "control_msgs")

(defun init ()
  (ros::load-ros-manifest "roseus")
  (ros::roseus "juggle-system")

  ;;
  ;; init variables
  ;;
  (setq *idle-diabolo-pitch* 0)
  (setq *idle-diabolo-yaw* 0)
  
  (setq *diabolo-system-input-arm* 0)
  (setq *diabolo-system-input-base* 0)
  
  ;;
  ;; Subscriber
  ;;
  ;; subscribe idle diabolo state
  (ros::create-nodehandle "idle")
  (ros::subscribe "calc_idle_diabolo_state/diabolo_state" std_msgs::Float64MultiArray
                  #'(lambda (msg) (progn
                                    (setq *idle-diabolo-pitch* (elt (send msg :data) 0))
                                    (setq *idle-diabolo-yaw* (elt (send msg :data) 1))
                                    ))
                  :groupname "idle")
  (ros::subscribe "joy" sensor_msgs::Joy
                  #'(lambda (msg) (progn
                                    (setq *stop-feedback* (elt (send msg :buttons) 14))
                                    ))
                  :groupname "idle")

  
  ;; subscribe joy
  (ros::create-nodehandle "joy")
  ;; (ros::subscribe "joy" sensor_msgs::Joy
  ;; 		  #'(lambda (msg) (setq *joy* (send msg :axes)))
	;; 	  :groupname "joy")

  ;; subscribe DiaboloSystem
  (ros::create-nodehandle "diabolo-system")
  (ros::subscribe "diabolo_system/diabolo_input" std_msgs::Float64MultiArray
                  #'(lambda (msg) (progn
                                    (setq *diabolo-system-input-arm* (elt (send msg :data) 0))
                                    (setq *diabolo-system-input-base* (elt (send msg :data) 1))
                                    ))
                  :groupname "diabolo-system")
  (ros::create-nodehandle "diabolo-system-simulate")
  (ros::subscribe "diabolo_system/diabolo_state" std_msgs::Float64MultiArray
                  #'(lambda (msg) (progn
                                    (setq *idle-diabolo-pitch* (elt (send msg :data) 0))
                                    (setq *idle-diabolo-yaw* (elt (send msg :data) 1))
                                    ))
                  :groupname "diabolo-system-simulate")

  ;;
  ;; Publisher
  ;;
  ;; publish whether idle or not
  (ros::advertise "idle" std_msgs::float64 1)

  ;; publish larm
  (ros::advertise "/l_arm_controller/follow_joint_trajectory/goal" control_msgs::FollowJointTrajectoryActionGoal 1)  

  ;;
  ;; init PR2
  ;;
  (load "package://pr2eus/pr2-interface.l")
  (pr2-init)
  
  (send *ri* :angle-vector (send *pr2* :reset-pose) 5000)
  (setq *ac* (pr2-interface-move-base-trajectory-action *ri*))
  
  (send *ri* :start-grasp)
  (send *pr2* :head-neck-p :joint-angle 50)

  (solve-ik)
  
  (objects (list *pr2*))
  
  (load "package://diabolo_pr2_201806/euslisp/utils.l")
  (load "package://diabolo_pr2_201806/euslisp/idle.l")
  )

(defun solve-ik ()
  (send *pr2* :reset-pose)
  (send *pr2* :start-grasp)  
  (send *pr2* :head-neck-p :joint-angle 50)
  
  (progn ;; default 500~900
    (setq *default-right-cube* (make-cube 100 100 100 :pos (float-vector 700 -120 1050)))
    (send *pr2* :rarm :inverse-kinematics *default-right-cube*)  
    (setq *default-left-cube* (make-cube 100 100 100 :pos (float-vector 700 120 1050)))
    (send *pr2* :larm :inverse-kinematics *default-left-cube*)   
    (setq *default-pos* (send *pr2* :angle-vector))
    )

  (progn  ;; idle
    (setq *idle-right-up-cube* (make-cube 100 100 100 :pos (float-vector 700 -150 1150)))
    (send *pr2* :rarm :inverse-kinematics *idle-right-up-cube*)  
    (setq *idle-left-down-cube* (make-cube 100 100 100 :pos (float-vector 700 150 950)))
    (send *pr2* :larm :inverse-kinematics *idle-left-down-cube*)    
    (setq *idle-1-pos* (send *pr2* :angle-vector))
    
    (setq *idle-right-down-cube* (make-cube 100 100 100 :pos (float-vector 700 -150 950)))
    (send *pr2* :rarm :inverse-kinematics *idle-right-down-cube*)
    (setq *idle-left-up-cube* (make-cube 100 100 100 :pos (float-vector 700 150 1150)))
    (send *pr2* :larm :inverse-kinematics *idle-left-up-cube*)   
    (setq *idle-2-pos* (send *pr2* :angle-vector))
    )
  
  (progn ;; roll
    (setq *roll-1-right-cube* (make-cube 100 100 100 :pos (float-vector 700 -50 950)))
    (send *pr2* :rarm :inverse-kinematics *roll-1-right-cube*)
    (setq *roll-1-left-cube* (make-cube 100 100 100 :pos (float-vector 700 250 950)))
    (send *pr2* :larm :inverse-kinematics *roll-1-left-cube*)   
    (setq *roll-1-pos* (send *pr2* :angle-vector))

    (setq *roll-2-right-cube* (make-cube 100 100 100 :pos (float-vector 700 -50 800)))
    (send *pr2* :rarm :inverse-kinematics *roll-2-right-cube*)  
    (setq *roll-2-left-cube* (make-cube 100 100 100 :pos (float-vector 700 250 800)))
    (send *pr2* :larm :inverse-kinematics *roll-2-left-cube*)    
    (setq *roll-2-pos* (send *pr2* :angle-vector))
    
    (setq *roll-3-right-cube* (make-cube 100 100 100 :pos (float-vector 700 -700 850)))
    (send *pr2* :rarm :inverse-kinematics *roll-3-right-cube*)  
    (setq *roll-3-left-cube* (make-cube 100 100 100 :pos (float-vector 700 -400 850)))
    (send *pr2* :larm :inverse-kinematics *roll-3-left-cube* :rotation-axis :z)    
    (setq *roll-3-pos* (send *pr2* :angle-vector))
    
    (send *pr2* :angle-vector *roll-2-pos*)
    (setq *roll-4-right-cube* (make-cube 100 100 100 :pos (float-vector 700 -150 750)))
    (send *pr2* :rarm :inverse-kinematics *roll-4-right-cube*)  
    (setq *roll-4-left-cube* (make-cube 100 100 100 :pos (float-vector 700 150 750)))
    (send *pr2* :larm :inverse-kinematics *roll-4-left-cube*)    
    (setq *roll-4-pos* (send *pr2* :angle-vector))
    )

  (progn ;; toss
    ;;(send *pr2* :reset-pose)
    (send *pr2* :angle-vector #f(50.0 16.191 44.0774 31.227 -101.578 295.967 -52.3229 -46.5188 -16.6466 44.8379 -31.9883 -101.951 -272.998 -97.2327 394.953 0.0 5.0))
    ;;(send *pr2* :head-neck-p :joint-angle 5)

    ;; start
    (setq *toss-start-right-cube* (make-cube 100 100 100 :pos (float-vector 500 -350 950))) ;; 500 -350 850)))
    (send *toss-start-right-cube* :rotate (* pi 0.5) :x)
    (send *toss-start-right-cube* :rotate (* pi 0.5) :z)
    (send *toss-start-right-cube* :rotate (* pi -0.4) :y)
    (send *pr2* :rarm :inverse-kinematics *toss-start-right-cube*)
    (setq *toss-start-left-cube* (make-cube 100 100 100 :pos (float-vector 500 350 950))) ;; 500 350 850)))
    (send *toss-start-left-cube* :rotate (* pi -0.5) :x)
    (send *toss-start-left-cube* :rotate (* pi -0.5) :z)
    (send *toss-start-left-cube* :rotate (* pi -0.4) :y)
    (send *pr2* :larm :inverse-kinematics *toss-start-left-cube*)
    (setq *toss-start-pos* (send *pr2* :angle-vector))
    #|
    (let ((avs (list)))
      (send *pr2* :rarm :inverse-kinematics (make-coords :pos #f(500.0 -150.0 1050.0) :rpy (float-vector pi/2 0 0)))
      (send *pr2* :larm :inverse-kinematics (make-coords :pos #f(500.0 150.0 1050.0) :rpy (float-vector -pi/2 0 0)))
      (setq avs (append avs (list (send *pr2* :angle-vector))))

      (send *pr2* :rarm :inverse-kinematics (make-coords :pos #f(500.0 -150.0 1050.0) :rpy (float-vector pi/2 -1 0)))
      (send *pr2* :larm :inverse-kinematics (make-coords :pos #f(500.0 150.0 1050.0) :rpy (float-vector -pi/2 -1 0)))
      (setq avs (append avs (list (send *pr2* :angle-vector))))

      (send *pr2* :rarm :inverse-kinematics (make-coords :pos #f(500.0 -350.0 950.0) :rpy (float-vector pi/2 -2.5 0)))
      (send *pr2* :larm :inverse-kinematics (make-coords :pos #f(500.0 350.0 950.0) :rpy (float-vector -pi/2 -2.5 0)))
      (setq avs (append avs (list (send *pr2* :angle-vector))))
      (setq *toss-start-avs* avs)
      )
    |#

    ;; goal
    (setq *toss-goal-right-cube* (make-cube 100 100 100 :pos (float-vector 500 -860 1150))) ;; 500 -730 1250
    (send *toss-goal-right-cube* :rotate (* pi 0.5) :x)
    (send *toss-goal-right-cube* :rotate (* pi 0.5) :z)
    (send *toss-goal-right-cube* :rotate (* pi -0.4) :y)
    ;;(send *toss-goal-right-cube* :rotate (* pi -0.2) :x)
    (send *pr2* :rarm :inverse-kinematics *toss-goal-right-cube*)
    (setq *toss-goal-left-cube* (make-cube 100 100 100 :pos (float-vector 500 860 1150))) ;; 500 730 1250
    (send *toss-goal-left-cube* :rotate (* pi -0.5) :x)
    (send *toss-goal-left-cube* :rotate (* pi -0.5) :z)
    (send *toss-goal-left-cube* :rotate (* pi -0.4) :y)
    ;;(send *toss-goal-left-cube* :rotate (* pi 0.2) :x)
    (send *pr2* :larm :inverse-kinematics *toss-goal-left-cube*)
    (setq *toss-goal-pos* (send *pr2* :angle-vector))

    #|
    ;; save
    (setq *toss-save-right-cube* (make-cube 100 100 100 :pos (float-vector 580 -320 1000))) ;; 580 -720
    (send *toss-save-right-cube* :rotate (* pi 0.5) :x)
    (send *toss-save-right-cube* :rotate (* pi 0.5) :z)
    (send *toss-save-right-cube* :rotate (* pi -0.4) :y)
    (send *pr2* :rarm :inverse-kinematics *toss-save-right-cube*)
    (setq *toss-save-left-cube* (make-cube 100 100 100 :pos (float-vector 580 320 1000))) ;; 580 720
    (send *toss-save-left-cube* :rotate (* pi -0.5) :x)
    (send *toss-save-left-cube* :rotate (* pi -0.5) :z)
    (send *toss-save-left-cube* :rotate (* pi -0.4) :y)
    (send *pr2* :larm :inverse-kinematics *toss-save-left-cube*)
    (setq *toss-save-pos* (send *pr2* :angle-vector))
    |#
    )
  )
