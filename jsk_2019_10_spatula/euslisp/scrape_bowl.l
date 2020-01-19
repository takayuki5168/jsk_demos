 #!/usr/bin/env roseus

;;hint use 543 l to put down the bowl


;;;;
;;;; make scraping movement with right hand, move the bowl with the left Hand until the spatula touches the bowl. 
;;;;
(ros::load-ros-package "jsk_2019_10_spatula")
(load "package://pr2eus/pr2-interface.l")
(ros::roseus "pr2_send_joints")
(ros::advertise "scrape_left_jacobian" jsk_2019_10_spatula::Jacobian 1)
(ros::advertise "scrape_right_jacobian" jsk_2019_10_spatula::Jacobian 1)
(ros::advertise "semantic_annotation" std_msgs::string 1)
(setq *ri* (instance pr2-interface :init))
(pr2-init t)

;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;define global variables;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;
(load "angle-vectors.l")


;;;;;;;;;;;;;;;;;;;;;;;;
;;function definitions;;
;;;;;;;;;;;;;;;;;;;;;;;;

(defun publish-semantic-annotation (annotation-str)
  (setq msg (instance std_msgs::string :init))
  (send msg :data annotation-str)
  (ros::ros-info "msg [~A]" (send msg :data))
  (ros::publish "semantic_annotation" msg)
  )

(defun publish-jacobi (arm)
  ;;get the line components of the jacobi
  (setq selection #f(1 0 0 0 0 0))
  (setq jacobi-x (transform selection jacobi))
  (setq selection #f(0 1 0 0 0 0)) 
  (setq jacobi-y (transform selection jacobi))
  (setq selection #f(0 0 1 0 0 0))
  (setq jacobi-z (transform selection jacobi))
  (setq selection #f(0 0 0 1 0 0))
  (setq jacobi-roll (transform selection jacobi))
  (setq selection #f(0 0 0 0 1 0))
  (setq jacobi-pitch (transform selection jacobi))
  (setq selection #f(0 0 0 0 0 1))
  (setq jacobi-yaw (transform selection jacobi))
  ;;pack the line components into Jacobian msg
  (setq msg (instance jsk_2019_10_spatula::Jacobian :init))
  (send msg :x jacobi-x)
  (send msg :y jacobi-y)
  (send msg :z jacobi-z)
  (send msg :roll jacobi-yaw)
  (send msg :pitch jacobi-pitch)
  (send msg :yaw jacobi-yaw)
  ;;print the msg
  ;;(ros::ros-info "msg [~A]" (send msg :x))
  ;;publish the msg
  (if (= arm 0)
      (ros::publish "scrape_left_jacobian" msg)
      (ros::publish "scrape_right_jacobian" msg)
      )
  )

;;get jacobian in world coordinate frame
(defun get-jacobi-l-world ()
  (setq link-list (send *pr2* :link-list (send (send *pr2* :larm :end-coords) :parent)))
  (setq joint-list (send-all link-list :joint))
  (setq jacobi
	(send *pr2* :calc-jacobian-from-link-list
	      link-list
	      :move-target (send *pr2* :larm :end-coords)
	      :transform-coords (make-coords) ;;in world coords, if left out it is in local coords
	      :rotation-axis t
	      ))
  ;;(format-array jacobi) ;; print
  (publish-jacobi 0)
  )


(defun get-jacobi-r-world ()
  (setq link-list (send *pr2* :link-list (send (send *pr2* :rarm :end-coords) :parent)))
  (setq joint-list (send-all link-list :joint))
  (setq jacobi
	(send *pr2* :calc-jacobian-from-link-list
	      link-list
	      :move-target (send *pr2* :rarm :end-coords)
	      :transform-coords (make-coords) ;;jacobian is in world coords
	      :rotation-axis t
	      ))
  ;;(format-array jacobi) ;; print
  (publish-jacobi 1)
 )

(defun move-left-arm (d)
  ;;(send *l-tcp* :translate (float-vector 0 0 (* -1 (* d 10)))) ;;new bowl
  (send *l-tcp* :translate (float-vector (* d 10) 0 0)) ;;new bowl
  (send *pr2* :larm :inverse-kinematics *l-tcp*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000 :larm-controller)
  (while (send *ri* :interpolatingp :larm-controller)
    (send *ri* :spin-once))
  )

;;define callback function
(defun string-cb (msg)
  (print (list 'cb (sys::thread-self) (send msg :data)))
  (if (= (send msg :data) 0)
      (print "everything perfect!");;no reaction, touching
      (move-left-arm (send msg :data))
      )
  )

;;define callback function
(defun schedule-scraping (msg)
  (setq label-published t)
  (print "schedule scraping now")
  (setq *label* (send msg :data))
  (setq i 0)
  (dolist (dirty *label*)
    (print dirty)
    (when (and dirty (not (= i 7))) ;;if piece is dirty and not 7
      (print "scrape, it is dirty!")
      (print i)
      (clean i)
      )
    (setq i (+ i 1))
    )
  )

(defun clean (i)
  (print "clean")
  ;;slight offset of 0.3 on the position as inverse-linematics fail for i=0 -> position=0
  ;;(setq position (+ (* 1.0 (mod i *n-bowl-position*)) 0.3))
  ;;(setq position (+ (* 1.0 (mod i *n-bowl-position*)) 1 ))
  (setq position (+ (mod i *n-bowl-position*) 1 ))

  ;;(setq step-size (/ 360 (* 4  *n-bowl-position*)))
  (setq step-size (/ 360 (* 4  5)))
  (setq omega (* (* (/ (- 90 (+ (* (* 1.0 position)  step-size) (/ step-size 2))) 360) 2) pi))
  (if (< i *n-bowl-position*)
      (setq type 0)
      (if (< i (* 2 *n-bowl-position*)) ;;elseif
	  (setq type 1)
	  (if (< i (* 3 *n-bowl-position*)) ;;elseif
	      (setq type 2)
	      (setq type 3) ;;else
	      )
	  )
    )
  ;;quick workaround to make experiment work
  ;;(setq oldtype -5)
  (unless (= oldtype type)
     (send *pr2* :angle-vector *av-base*)	  
     (send *ri* :angle-vector (send *pr2* :angle-vector) 500 :rarm-controller)	  
     (send *ri* :wait-interpolation)	  	
     )
  (position-left-arm omega)
  ;;(unix:usleep 500000)
  (scrape-wall 1000 type position)
  ;;(unix:usleep 500000)
  ;;(unix:usleep 1000000)
  ;;(scrape-wall 4000 type position)
  (setq oldtype type)
  )

(defun position-left-arm (omega)
  (print "position left arm")
  ;;(setq r 97.5)
  (setq r 125)
  (send *l-tcp* :translate (float-vector r 0 0)) ;;move KS into middle of bowl
  (send *l-tcp* :translate (float-vector (- 0 (* r (cos omega))) (- 0 (* r (sin omega))) 0) ) 
  (send *l-tcp* :rotate omega :z)
  (send *pr2* :larm :inverse-kinematics *l-tcp*)
  (if (boundp '*irtviewer*)
      (send *irtviewer* :draw-objects))
  (unless (arm-collision-check)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000 :larm-controller)
    (send *ri* :wait-interpolation)
    )
  ;;move KS back into original Position
  (send *l-tcp* :rotate (- omega) :z) ;;rotate back
  (send *l-tcp* :translate (float-vector (+ 0 (* r (cos omega))) (+ 0 (* r (sin omega))) 0) )
  (send *l-tcp* :translate (float-vector (- 0 r) 0 0)) ;;move KS into middle
  )

;;collision check function
(defun arm-collision-check ()
    (send *pr2* :self-collision-check :pairs
          (append
           (mapcar
            #'(lambda (l) (cons l (car (send *pr2* :larm :links))))
            (all-child-links (send *pr2* :larm-elbow-p :parent-link)))
           (mapcar
            #'(lambda (l) (cons l (car (send *pr2* :rarm :links))))
            (all-child-links (send *pr2* :rarm-elbow-p :parent-link))))
          ))

;;reaction to feedback_touching, the left arm adjusts the bowl in order to make the spatula touch it
(defun move-left-arm (msg)
  (print "move left arm!!")
  (setq d (send msg :data))
  ;;(send *l-tcp* :translate (float-vector 0 0 (* -1 (* d 10)))) ;;new bowl
  ;;for scraping the wall
  (print "moving")
  (print d)
  (send *l-tcp* :translate (float-vector (* d 10) 0 0)) ;;new bowl
  (send *pr2* :larm :inverse-kinematics *l-tcp*)
  (unless (arm-collision-check)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 500 :larm-controller)
    )
  (while (send *ri* :interpolatingp :larm-controller)
    (send *ri* :spin-once))
  )



;;scraping movement of the right arm to scrape the wall of the bowl
(defun scrape-wall (speed type bowl-position)
  ;;decide where to scrape
  ;;scrape the side of handle
  (get-jacobi-l-world)
  (when (= type 3)
    (setq *av1-wall* #f(324.807 49.5519 26.3404 123.787 -101.985 -6.62817 -29.4461 135.656 -31.7304 18.8068 -77.3953 -120.967 -93.5516 -70.375 265.871 0.006701 65.0))
    (setq *av2-wall* #f(324.807 49.5519 26.3404 123.787 -101.985 -6.62817 -29.4461 135.656 -23.0904 12.2131 -93.6752 -117.796 -100.776 -70.1569 275.7 0.006701 65.0))
    (setq *av4-wall* #f(324.807 49.5519 26.3404 123.787 -101.985 -6.62817 -29.4461 135.656 -27.7253 2.64068 -84.0543 -120.025 -86.5879 -74.2459 273.116 0.006701 65.0))
    (setq *av3-wall* #f(324.807 49.5519 26.3404 123.787 -101.985 -6.62817 -29.4461 135.656 -31.3168 2.62658 -102.552 -120.701 -94.8818 -58.5094 270.153 0.006701 65.0))
    )
  ;;scrape the opposite of handle
  (when (= type 1)
    (setq *av4-wall* #f(324.807 49.5519 26.3404 123.787 -101.985 -6.62817 -29.4461 135.656 -5.2312 -8.16999 -94.3449 -111.498 -81.3893 -105.243 106.913 0.006701 65.0))
    (setq *av1-wall* #f(324.807 49.5519 26.3404 123.787 -101.985 -6.62817 -29.4461 135.656 -29.8983 -3.39327 -90.0018 -121.542 -90.9323 -91.9124 85.3999 0.006701 65.0))
    (setq *av2-wall* #f(324.807 49.5519 26.3404 123.787 -101.985 -6.62817 -29.4461 135.656 -16.702 8.37012 -105.407 -119.426 -110.721 -85.9933 98.9265 0.006701 65.0))
    (setq *av3-wall* #f(324.807 49.5519 26.3404 123.787 -101.985 -6.62817 -29.4461 135.656 -18.4734 2.28735 -100.041 -120.695 -100.617 -82.9872 105.151 0.006701 65.0))
    )
  ;;scrape the back, close to robot
  (when (= type 2)
    (setq *av4-wall* #f(324.807 49.5519 26.3404 123.787 -101.985 -6.62817 -29.4461 135.656 -13.1883 -7.75134 -95.2939 -99.6978 -61.4526 -89.0247 170.473 0.006701 65.0))
    (setq *av1-wall* #f(324.807 7.7442 18.0848 125.631 -43.3995 2.08124 -110.359 127.308 -12.4932 -1.88 -89.825 -111.277 -66.0183 -86.2273 189.076 0.006701 65.0))
    (setq *av2-wall* #f(324.807 49.5519 26.3404 123.787 -101.985 -6.62817 -29.4461 135.656 -17.3638 11.3774 -94.0493 -112.216 -80.6065 -75.4007 180.136 0.006701 65.0))
    (setq *av3-wall* #f(324.807 49.5519 26.3404 123.787 -101.985 -6.62817 -29.4461 135.656 -10.9662 2.0885 -98.4725 -114.042 -90.5557 -85.5465 194.766 0.006701 65.0))
    )
  ;;scrape the front, away from robot
  (when (= type 0)
    (setq *av1-wall* #f(49.9246 49.5519 26.3404 123.787 -101.985 -6.62817 -29.4461 135.656 -29.0453 -7.74575 -88.6816 -121.542 -105.995 -76.5766 369.152 0.006701 65.0))
    (setq *av2-wall* #f(324.807 49.5519 26.3404 123.787 -101.985 -6.62817 -29.4461 135.656 -21.1492 22.1516 -90.049 -121.542 -141.64 -92.8129 367.131 0.006701 65.0))
    (setq *av4-wall* #f(324.807 49.5519 26.3404 123.787 -101.985 -6.62817 -29.4461 135.656 -14.741 -0.964581 -84.7095 -117.467 -81.8724 -96.6787 368.785 0.006701 65.0))
    (setq *av3-wall*  #f(324.807 49.5519 26.3404 123.787 -101.985 -6.62817 -29.4461 135.656 -18.3619 10.089 -92.1263 -113.945 -100.031 -88.7665 367.505 0.006701 65.0))
    )

  

  ;;start with home pos
  (unless (= oldtype type)
    (send *pr2* :angle-vector *av4-wall*)
    (unless (arm-collision-check)
      (send *ri* :angle-vector (send *pr2* :angle-vector) speed :rarm-controller)
      ;;(send *ri* :wait-interpolation)
        (while (send *ri* :interpolatingp :rarm-controller)
	  (send *ri* :spin-once)
	  (ros::spin-once);;to see if chatter published
	  (get-jacobi-r-world)
	  (if (send *ri* :simulation-modep)  
	      (send *ri* :robot-interface-simulation-callback))
	  )
	
      )
    )
  
  
  (send *pr2* :angle-vector *av1-wall*)
  (unless (arm-collision-check)
    (publish-semantic-annotation (format nil "av1wall_~a_~a_start" type bowl-position))
    (send *ri* :angle-vector (send *pr2* :angle-vector) speed :rarm-controller)
    ;;(send *ri* :wait-interpolation)
    (while (send *ri* :interpolatingp :rarm-controller)
      (send *ri* :spin-once)
      (ros::spin-once);;to see if chatter published
      (get-jacobi-r-world)
      (if (send *ri* :simulation-modep)
	  (send *ri* :robot-interface-simulation-callback))
      )
    (publish-semantic-annotation (format nil "av1wall_~a_~a_end" type bowl-position))
    )
  
  
  (send *pr2* :angle-vector *av2-wall*)
  (unless (arm-collision-check)
    (publish-semantic-annotation (format nil "av2wall_~a_~a_start" type bowl-position))
    (send *ri* :angle-vector (send *pr2* :angle-vector) speed :rarm-controller)
    ;;(send *ri* :wait-interpolation)
    (while (send *ri* :interpolatingp :rarm-controller)
      (send *ri* :spin-once)
      (ros::spin-once);;to see if chatter published
      (get-jacobi-r-world)
      (if (send *ri* :simulation-modep)
	  (send *ri* :robot-interface-simulation-callback))
      )
    (publish-semantic-annotation (format nil "av2wall_~a_~a_end" type bowl-position))
    
    )


  (send *pr2* :angle-vector *av3-wall*)
  (unless (arm-collision-check)
    (publish-semantic-annotation (format nil "av3wall_~a_~a_start" type bowl-position))
    (send *ri* :angle-vector (send *pr2* :angle-vector) speed :rarm-controller)
    ;;(send *ri* :wait-interpolation)
    (while (send *ri* :interpolatingp :rarm-controller)
      (send *ri* :spin-once)
      (ros::spin-once);;to see if chatter published
      (get-jacobi-r-world)
      (if (send *ri* :simulation-modep)
	  (send *ri* :robot-interface-simulation-callback))
      )
    (publish-semantic-annotation (format nil "av3wall_~a_~a_end" type bowl-position))
    
    )

  
  (send *pr2* :angle-vector *av4-wall*)
  (unless (arm-collision-check)
    (publish-semantic-annotation (format nil "av4wall_~a_~a_start" type bowl-position))
    (send *ri* :angle-vector (send *pr2* :angle-vector) speed :rarm-controller)
    ;;(send *ri* :wait-interpolation)
    (while (send *ri* :interpolatingp :rarm-controller)
      (send *ri* :spin-once)
      (ros::spin-once);;to see if chatter published
      (get-jacobi-r-world)
      (if (send *ri* :simulation-modep)
	  (send *ri* :robot-interface-simulation-callback))
      )
    (publish-semantic-annotation (format nil "av4wall_~a_~a_end" type bowl-position))
    
    )

  ;;(publish-semantic-annotation (format nil "end_~a_~a" type bowl-position))
  )


(defun prepare-robot ()
  (send *pr2* :angle-vector  *av-reset-pose-high*)
  (if (boundp '*irtviewer*)
    (send *irtviewer* :draw-objects))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  
  ;; move robot into initial position
  (send *pr2* :rarm :shoulder-p :joint-angle -20)
  (send *pr2* :larm :shoulder-p :joint-angle 50)
  (send *pr2* :rarm :shoulder-r :joint-angle -130)
  (send *pr2* :larm :shoulder-r :joint-angle 120)
  (send *pr2* :larm :elbow-p :joint-angle -100)
  (send *pr2* :rarm :elbow-p :joint-angle -70)
  (send *pr2* :rarm :elbow-r :joint-angle 0)
  (send *pr2* :rarm :wrist-p :joint-angle -40)
  (send *pr2* :larm :wrist-r :joint-angle 30)
  (send *pr2* :head :neck-p :joint-angle 72)
  
  (if (boundp '*irtviewer*)
      (send *irtviewer* :draw-objects))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  
  ;; hand the robot the spatula
  (send *pr2* :stop-grasp)
  (send *ri* :stop-grasp)
  (send *ri* :wait-interpolation)
  (unix:sleep 5)
  (send *pr2* :start-grasp :rarm)
  (send *ri* :start-grasp :rarm :gain 0.05)
  (send *ri* :wait-interpolation)

  (publish-semantic-annotation (format nil "av_new_bowl"))
  
  ;;start position
  (send *pr2* :angle-vector *av-newbowl*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000 :larm-controller)
  (send *ri* :wait-interpolation)

  (publish-semantic-annotation (format nil "hand_bowl"))
  
  ;;hand the robot the bowl
  (unix:sleep 2)
  (send *pr2* :start-grasp :larm)
  (send *ri* :start-grasp :larm :gain 0.05)
  (send *ri* :wait-interpolation)

  ;;one vector inbetween the start to avoid hitting the bowl with the spatula  
  (send *pr2* :angle-vector *av-r-approach*) 
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1500 :rarm-controller) 
  (send *ri* :wait-interpolation)
  (send *pr2* :angle-vector *av3*) 
  (send *ri* :angle-vector (send *pr2* :angle-vector) 700 :larm-controller)
  (send *ri* :wait-interpolation)
  (setq *l-tcp* (send (send *pr2* :larm :end-coords) :copy-worldcoords))
  (setq *ltargetframe* (send *pr2* :larm :end-coords))
  )

(defun transfer-dough-2 (mode)
  (if (= mode 0)  (send *pr2* :angle-vector *av-transfer-1-r*)  (send *pr2* :angle-vector *av-transfer-1b-r*))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  (if (= mode 0)  (send *pr2* :angle-vector *av-transfer-2-r*)  (send *pr2* :angle-vector *av-transfer-2b-r*))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  (if (= mode 0)  (send *pr2* :angle-vector *av-transfer-3-r*)  (send *pr2* :angle-vector *av-transfer-3b-r*))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  )

(defun get-dense-ptcloud()
  (ros::service-call "passthrough_points/request" (instance std_srvs::EmptyRequest :init))
  (send *pr2* :angle-vector *look-front*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  (unix:usleep 10000) ;;sleep 1/100 of a second to make sure /tf is fresh
  ;;passthrough the pointcloud data
  (ros::service-call "passthrough_points/request" (instance std_srvs::EmptyRequest :init))
  (publish-semantic-annotation (format nil "vision_0"))
  
  (send *pr2* :angle-vector *look-right*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  (unix:usleep 10000) ;;sleep 1/100 of a second to make sure /tf is fresh
  (ros::service-call "passthrough_points/request" (instance std_srvs::EmptyRequest :init))
  (publish-semantic-annotation (format nil "vision_1"))

  (send *pr2* :angle-vector *look-back*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  ;;(unix:sleep 1)
  (ros::service-call "passthrough_points/request" (instance std_srvs::EmptyRequest :init))
  (publish-semantic-annotation (format nil "vision_2"))

  (send *pr2* :angle-vector *look-left*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  (unix:usleep 10000) ;;sleep 1/100 of a second to make sure /tf is fresh
  (ros::service-call "passthrough_points/request" (instance std_srvs::EmptyRequest :init))
  (publish-semantic-annotation (format nil "vision_3"))

  (send *pr2* :angle-vector *av-snapshot*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  (unix:usleep 10000) ;;sleep 1/100 of a second to make sure /tf is fresh
  (ros::service-call "passthrough_points/request" (instance std_srvs::EmptyRequest :init))
  (publish-semantic-annotation (format nil "vision_end"))
  )

(defun get-pctloud-piece (i)
  (clean i)
  (get-dense-ptcloud)
  )

(defun get-pctloud-all-clean ()
  ;;clean the whole bowl
  (dolist (i (list 0 1 2 3 4 5 6 8 9 10 11 12 13 14 15)) ;;7 is front, does not need to be cleaned!
    (clean i)
    ;;(unix:usleep 2000000)
  ;;(get-dense-ptcloud)
    ))

  (defun prepare-bowl-transfer ()
    (send *pr2* :angle-vector *av3*)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    (setq *joint-angle-time-l* (list 1000 1000 1000 1000 1000 1000))
    (setq *joint-angle-list-l* (list *av-transfer-1-l*  *av-transfer-2-l* *av-transfer-3-l* *av-transfer-4-l*  *av-transfer-5-l* *av-transfer-6-l*))
    (send *ri* :angle-vector-sequence *joint-angle-list-l* *joint-angle-time-l*)
    (send *ri* :wait-interpolation)
    )

  (defun prepare-bowl-transfer-backwards ()
    (setq *joint-angle-time-l-back* (list 1000 1000 1000 1000 1000 1000))
    (setq *joint-angle-list-l-back* (list *av-transfer-6-l*  *av-transfer-5-l* *av-transfer-4-l* *av-transfer-3-l*  *av-transfer-2-l* *av-transfer-1-l*))
    (send *ri* :angle-vector-sequence *joint-angle-list-l-back* *joint-angle-time-l-back*)
    (send *ri* :wait-interpolation)
    (send *pr2* :angle-vector *av3*)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    )
  
  (defun transfer-dough-13-1 ()
    (get-jacobi-l-world)
    (get-jacobi-r-world)
    ;;(setq *joint-angle-time-l* (list 1000 1000 1000 1000 1000 1000 1000))
    ;;(setq *joint-angle-list-l* (list *av-transfer-start-pos* *av-transfer-corner-up* *av-transfer-back-middle* *av-transfer-corner* *av-scrape-down-inb*  *av-transfer-front* *av-transfer-finish*))
    ;;(send *ri* :angle-vector-sequence *joint-angle-list-l* *joint-angle-time-l*)
    ;;(send *ri* :wait-interpolation)
    
    (send *pr2* :angle-vector *av-transfer-1-r*)
    (publish-semantic-annotation (format nil "av1transfer_start"))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    (publish-semantic-annotation (format nil "av1transfer_end"))
    (get-jacobi-r-world)
    
    (send *pr2* :angle-vector *av-transfer-2-r*)
    (publish-semantic-annotation (format nil "av2transfer_start"))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    (publish-semantic-annotation (format nil "av2transfer_end"))
    (get-jacobi-r-world)
    
    (send *pr2* :angle-vector *av-transfer-3-r*)
    (publish-semantic-annotation (format nil "av3transfer_start"))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    (publish-semantic-annotation (format nil "av3transfer_end"))
    (get-jacobi-r-world)
    
    (send *pr2* :angle-vector *av-transfer-4-r*)
    (publish-semantic-annotation (format nil "av4transfer_start"))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    (publish-semantic-annotation (format nil "av4transfer_end"))
    (get-jacobi-r-world)
    
    (send *pr2* :angle-vector  *av-transfer-5-r*)
    (publish-semantic-annotation (format nil "av5transfer_start"))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    (publish-semantic-annotation (format nil "av5transfer_end"))
    (get-jacobi-r-world)
    
    (send *pr2* :angle-vector *av-transfer-6-r*)
    (publish-semantic-annotation (format nil "av6transfer_start"))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    (publish-semantic-annotation (format nil "av6transfer_end"))
    (get-jacobi-r-world)
    
    (send *pr2* :angle-vector *av-transfer-7-r*)
    (publish-semantic-annotation (format nil "av7transfer_start"))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)	
    (send *ri* :wait-interpolation)
    (publish-semantic-annotation (format nil "av7transfer_end"))
    (get-jacobi-r-world)
    )

  (defun transfer-dough-13-1-short ()
    (get-jacobi-l-world)
    (get-jacobi-r-world)
    ;;(setq *joint-angle-time-l* (list 1000 1000 1000 1000 1000 1000 1000))
    ;;(setq *joint-angle-list-l* (list *av-transfer-start-pos* *av-transfer-corner-up* *av-transfer-back-middle* *av-transfer-corner* *av-scrape-down-inb*  *av-transfer-front* *av-transfer-finish*))
    ;;(send *ri* :angle-vector-sequence *joint-angle-list-l* *joint-angle-time-l*)
    ;;(send *ri* :wait-interpolation)
    
    (send *pr2* :angle-vector *av-transfer-1-r*)
    (publish-semantic-annotation (format nil "av1transfer_start"))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    (publish-semantic-annotation (format nil "av1transfer_end"))
    (get-jacobi-r-world)
    
    (send *pr2* :angle-vector *av-transfer-2-r-short*)
    (publish-semantic-annotation (format nil "av2transfer_start"))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    (publish-semantic-annotation (format nil "av2transfer_end"))
    (get-jacobi-r-world)
    
    (send *pr2* :angle-vector *av-transfer-3-r-short*)
    (publish-semantic-annotation (format nil "av3transfer_start"))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    (publish-semantic-annotation (format nil "av3transfer_end"))
    (get-jacobi-r-world)
    
    (send *pr2* :angle-vector *av-transfer-4-r-short*)
    (publish-semantic-annotation (format nil "av4transfer_start"))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    (publish-semantic-annotation (format nil "av4transfer_end"))
    (get-jacobi-r-world)
    
    (send *pr2* :angle-vector  *av-transfer-5-r-short*)
    (publish-semantic-annotation (format nil "av5transfer_start"))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    (publish-semantic-annotation (format nil "av5transfer_end"))
    (get-jacobi-r-world)
    
    (send *pr2* :angle-vector *av-transfer-6-r-short*)
    (publish-semantic-annotation (format nil "av6transfer_start"))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    (publish-semantic-annotation (format nil "av6transfer_end"))
    (get-jacobi-r-world)
    
    (send *pr2* :angle-vector *av-transfer-7-r-short*)
    (publish-semantic-annotation (format nil "av7transfer_start"))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)	
    (send *ri* :wait-interpolation)
    (publish-semantic-annotation (format nil "av7transfer_end"))
    (get-jacobi-r-world)
    )

    (defun transfer-dough-13-1-middle ()
    (get-jacobi-l-world)
    (get-jacobi-r-world)
    ;;(setq *joint-angle-time-l* (list 1000 1000 1000 1000 1000 1000 1000))
    ;;(setq *joint-angle-list-l* (list *av-transfer-start-pos* *av-transfer-corner-up* *av-transfer-back-middle* *av-transfer-corner* *av-scrape-down-inb*  *av-transfer-front* *av-transfer-finish*))
    ;;(send *ri* :angle-vector-sequence *joint-angle-list-l* *joint-angle-time-l*)
    ;;(send *ri* :wait-interpolation)
    
    (send *pr2* :angle-vector *av-transfer-1-r*)
    (publish-semantic-annotation (format nil "av1transfer_start"))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    (publish-semantic-annotation (format nil "av1transfer_end"))
    (get-jacobi-r-world)
    
    (send *pr2* :angle-vector *av-transfer-2-r-middle*)
    (publish-semantic-annotation (format nil "av2transfer_start"))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    (publish-semantic-annotation (format nil "av2transfer_end"))
    (get-jacobi-r-world)
    
    (send *pr2* :angle-vector *av-transfer-3-r-middle*)
    (publish-semantic-annotation (format nil "av3transfer_start"))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    (publish-semantic-annotation (format nil "av3transfer_end"))
    (get-jacobi-r-world)
    
    (send *pr2* :angle-vector *av-transfer-4-r-middle*)
    (publish-semantic-annotation (format nil "av4transfer_start"))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    (publish-semantic-annotation (format nil "av4transfer_end"))
    (get-jacobi-r-world)
    
    (send *pr2* :angle-vector  *av-transfer-5-r-middle*)
    (publish-semantic-annotation (format nil "av5transfer_start"))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    (publish-semantic-annotation (format nil "av5transfer_end"))
    (get-jacobi-r-world)
    
    (send *pr2* :angle-vector *av-transfer-6-r-middle*)
    (publish-semantic-annotation (format nil "av6transfer_start"))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    (publish-semantic-annotation (format nil "av6transfer_end"))
    (get-jacobi-r-world)
    
    (send *pr2* :angle-vector *av-transfer-7-r-middle*)
    (publish-semantic-annotation (format nil "av7transfer_start"))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)	
    (send *ri* :wait-interpolation)
    (publish-semantic-annotation (format nil "av7transfer_end"))
    (get-jacobi-r-world)
    )

  (defun transfer-dough-13-1-right ()
    (get-jacobi-l-world)
    (get-jacobi-r-world)
    ;;(setq *joint-angle-time-l* (list 1000 1000 1000 1000 1000 1000 1000))
    ;;(setq *joint-angle-list-l* (list *av-transfer-start-pos* *av-transfer-corner-up* *av-transfer-back-middle* *av-transfer-corner* *av-scrape-down-inb*  *av-transfer-front* *av-transfer-finish*))
    ;;(send *ri* :angle-vector-sequence *joint-angle-list-l* *joint-angle-time-l*)
    ;;(send *ri* :wait-interpolation)
    
    (send *pr2* :angle-vector *av-transfer-1-r*)
    (publish-semantic-annotation (format nil "av1transfer_start"))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    (publish-semantic-annotation (format nil "av1transfer_end"))
    (get-jacobi-r-world)
    
    (send *pr2* :angle-vector *av-transfer-2-r-right*)
    (publish-semantic-annotation (format nil "av2transfer_start"))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    (publish-semantic-annotation (format nil "av2transfer_end"))
    (get-jacobi-r-world)
    
    (send *pr2* :angle-vector *av-transfer-3-r-right*)
    (publish-semantic-annotation (format nil "av3transfer_start"))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    (publish-semantic-annotation (format nil "av3transfer_end"))
    (get-jacobi-r-world)
    
    (send *pr2* :angle-vector *av-transfer-4-r-right*)
    (publish-semantic-annotation (format nil "av4transfer_start"))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    (publish-semantic-annotation (format nil "av4transfer_end"))
    (get-jacobi-r-world)
    
    (send *pr2* :angle-vector  *av-transfer-5-r-right*)
    (publish-semantic-annotation (format nil "av5transfer_start"))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    (publish-semantic-annotation (format nil "av5transfer_end"))
    (get-jacobi-r-world)
    
    (send *pr2* :angle-vector *av-transfer-6-r-right*)
    (publish-semantic-annotation (format nil "av6transfer_start"))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    (publish-semantic-annotation (format nil "av6transfer_end"))
    (get-jacobi-r-world)
    
    (send *pr2* :angle-vector *av-transfer-7-r-right*)
    (publish-semantic-annotation (format nil "av7transfer_start"))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)	
    (send *ri* :wait-interpolation)
    (publish-semantic-annotation (format nil "av7transfer_end"))
    (get-jacobi-r-world)
    )

   (defun transfer-dough-13-1-2 ()
    (get-jacobi-l-world)
    (get-jacobi-r-world)
    ;;(setq *joint-angle-time-l* (list 1000 1000 1000 1000 1000 1000 1000))
    ;;(setq *joint-angle-list-l* (list *av-transfer-start-pos* *av-transfer-corner-up* *av-transfer-back-middle* *av-transfer-corner* *av-scrape-down-inb*  *av-transfer-front* *av-transfer-finish*))
    ;;(send *ri* :angle-vector-sequence *joint-angle-list-l* *joint-angle-time-l*)
    ;;(send *ri* :wait-interpolation)
    
    (send *pr2* :angle-vector *av-transfer-1-r*)
    (publish-semantic-annotation (format nil "av1transfer_start"))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    (publish-semantic-annotation (format nil "av1transfer_end"))
    (get-jacobi-r-world)
    
    (send *pr2* :angle-vector *av-transfer-2-r-2*)
    (publish-semantic-annotation (format nil "av2transfer_start"))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    (publish-semantic-annotation (format nil "av2transfer_end"))
    (get-jacobi-r-world)
    
    (send *pr2* :angle-vector *av-transfer-3-r-2*)
    (publish-semantic-annotation (format nil "av3transfer_start"))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    (publish-semantic-annotation (format nil "av3transfer_end"))
    (get-jacobi-r-world)
    
    (send *pr2* :angle-vector *av-transfer-4-r-2*)
    (publish-semantic-annotation (format nil "av4transfer_start"))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    (publish-semantic-annotation (format nil "av4transfer_end"))
    (get-jacobi-r-world)
    
    (send *pr2* :angle-vector  *av-transfer-5-r-2*)
    (publish-semantic-annotation (format nil "av5transfer_start"))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    (publish-semantic-annotation (format nil "av5transfer_end"))
    (get-jacobi-r-world)
    
    (send *pr2* :angle-vector *av-transfer-6-r-2*)
    (publish-semantic-annotation (format nil "av6transfer_start"))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    (publish-semantic-annotation (format nil "av6transfer_end"))
    (get-jacobi-r-world)
    
    (send *pr2* :angle-vector *av-transfer-7-r-2*)
    (publish-semantic-annotation (format nil "av7transfer_start"))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)	
    (send *ri* :wait-interpolation)
    (publish-semantic-annotation (format nil "av7transfer_end"))
    (get-jacobi-r-world)
    )
  

   (defun demo ()
     (print "start demo now")
  ;;(prepare-robot)
  (setq *l-tcp* (send (send *pr2* :larm :end-coords) :copy-worldcoords))
  (get-dense-ptcloud)
  ;;once the result is published, the scraping can start planned with the labels
  (while (not label-published)
    (ros::spin-once);;to see if labels where published
    )
  )

(defun grasp-cup ()
  (send *pr2* :stop-grasp :rarm)
  (send *ri* :stop-grasp :rarm)
  (send *pr2* :angle-vector  *av1-cup*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  (unix:sleep 2)
  (send *pr2* :angle-vector  *av2-cup*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  (send *pr2* :start-grasp :rarm)
  (send *ri* :start-grasp :rarm :gain 0.05)
  (send *pr2* :angle-vector  *av3-cup*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  )

(defun adjust-force (msg)
  (print msg.data)
  (setq gain msg.data)
  (setq av-diff (v- av-20mm av-0mm))
  (setq adjustment-av (scale gain av-diff))
  (setq adjusted-av (v+ av-0mm adjustment-av))
  )


;;;;;;;;;;;;;;;;
;;subcriptions;;
;;;;;;;;;;;;;;;;
(ros::subscribe "pcl_nodelet/dirt_label" jsk_2019_10_spatula::BoolArray #'schedule-scraping)
(ros::subscribe "force_gain"  std_msgs::float64 #'adjust-force)
;;(ros::subscribe "endeffector_force" jsk_2019_10_spatula::Force #'adapt-to-force)
;;(ros::subscribe "feedback_touching" std_msgs::float64 #'string-cb)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;fixed movement in beginning;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;(demo)

;;(prepare-robot)

;;can be used to gather the vision data (make sure to launch launchfiles first)
;;clean one specific part of bowl (0,1,2,3,4,5,6, ,8,9,10,11,12,13,14,15)
;;(setq i 0)
;;(clean i)


;;can be used to gather the effort data
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
#|
(setq i 0)
(setq position (+ (* 1.0 (mod i *n-bowl-position*)) 1 ))
(setq step-size (/ 360 (* 4  5)))
(setq omega (* (* (/ (- 90 (+ (* position  step-size) (/ step-size 2))) 360) 2) pi))
(position-left-arm omega)
|#


#|
(prepare-robot)
(setq *l-tcp* (send (send *pr2* :larm :end-coords) :copy-worldcoords))
(get-dense-ptcloud)
;;once the result is published, the scraping can start planned with the labels

(while (not label-published)
  (ros::spin-once);;to see if labels where published
  )
|#


#|
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; transfer the dough to the other bowl ;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;(send *ri* :go-pos-unsafe 0.4 0 0)

(send *pr2* :angle-vector *av3*)
(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
(send *ri* :wait-interpolation)

(send *pr2* :angle-vector *av-transfer-1-l*)
(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
(send *ri* :wait-interpolation)

(send *pr2* :angle-vector *av-transfer-2-l*)
(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
(send *ri* :wait-interpolation)

(send *pr2* :angle-vector *av-transfer-3-l*)
(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
(send *ri* :wait-interpolation)

(send *pr2* :angle-vector *av-transfer-1-r*)
(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
(send *ri* :wait-interpolation)


(setq *joint-angle-time-l* (list 1000 1000 1000)) 
(setq *joint-angle-list-l* (list *av-transfer-1-l*  *av-transfer-2-l* *av-transfer-3-l*))
(send *ri* :angle-vector-sequence *joint-angle-list-l* *joint-angle-time-l*)
(send *ri* :wait-interpolation)

;;(setq *joint-angle-time* (list 2000 2000 2000 2000 2000 2000 ))
(setq *joint-angle-time* (list 1000 1000 1000 1000 1000 1000 )) 
(setq *joint-angle-list* (list *av-transfer-3-l*  *av-transfer-2-r* *av-transfer-3-r* *av-transfer-3-l*  *av-transfer-2b-r* *av-transfer-3b-r*))
(dotimes (i 3)
 (send *ri* :angle-vector-sequence *joint-angle-list* *joint-angle-time*)
(send *ri* :wait-interpolation)
  )

(setq *joint-angle-time-r* (list 1000 1000 1000 1000 1000 1000)) 
(setq *joint-angle-list-r* (list *av-transfer-1-r*  *av-transfer-2-r* *av-transfer-3-r* *av-transfer-1b-r*  *av-transfer-2b-r* *av-transfer-3b-r*))
(send *ri* :angle-vector-sequence *joint-angle-list-r* *joint-angle-time-r*)

(dotimes (i 3)
(setq *joint-angle-time-ra* (list 700 700 700 400)) 
(setq *joint-angle-list-ra* (list *av-transfer-3-l*  *av-transfer-2-r* *av-transfer-23-r* *av-transfer-3-r*))
(send *ri* :angle-vector-sequence *joint-angle-list-ra* *joint-angle-time-ra*)
(send *ri* :wait-interpolation)
(unix:sleep 1)
(setq *joint-angle-time-rb* (list 700 700 700)) 
(setq *joint-angle-list-rb* (list *av-transfer-3-l*  *av-transfer-2b-r* *av-transfer-3b-r*))
(send *ri* :angle-vector-sequence *joint-angle-list-rb* *joint-angle-time-rb*)
(send *ri* :wait-interpolation)
(unix:sleep 1)
  )
|#
