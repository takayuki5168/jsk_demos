
#!/usr/bin/env roseus
;;;;
;;;; make scraping movement with right hand, move the bowl with the left Hand until the spatula touches the bowl. 
;;;;

(load "package://pr2eus/pr2-interface.l")
(ros::roseus "pr2_send_joints")
(ros::advertise "start_scraping" std_msgs::bool 1)

(setq *ri* (instance pr2-interface :init))
(pr2-init t)


#|
(defun move-left-arm (d)
  (print "move left arm!!")
  (send *l-tcp* :translate (float-vector 0 0 (* -1 (* d 10)))) ;;new bowl
  (send *pr2* :inverse-kinematics
	*l-tcp*
	:move-target *ltargetframe*
	:use-torso nil
	)
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
|#


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
  (send *l-tcp* :translate (float-vector 0 0 (* -1 (* d 10)))) ;;new bowl
  (send *pr2* :larm :inverse-kinematics *l-tcp*)
  (unless (arm-collision-check)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 100 :larm-controller)
    )
  (while (send *ri* :interpolatingp :larm-controller)
    (send *ri* :spin-once))
  )

;;(ros::subscribe "feedback_touching" std_msgs::float64 #'string-cb)
(ros::subscribe "feedback_touching" std_msgs::float64 #'move-left-arm)

;;start robot from reset position
(send *pr2* :reset-pose)
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

(if (boundp '*irtviewer*)    
    (send *irtviewer* :draw-objects))
(send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
(send *ri* :wait-interpolation)


;; hand the robot the spatula
(send *pr2* :stop-grasp)
(send *ri* :stop-grasp)
(send *ri* :wait-interpolation)
;;(unix:sleep 5)
(send *pr2* :start-grasp :rarm)
(send *ri* :start-grasp :rarm :gain 0.05)
(send *ri* :wait-interpolation)

;;define angle-vectors for movement
;;send *ri* :state :potentio-vector
;;(objects *r-tcp*)
(setq *av1* #f(49.9331 56.1185 28.9347 130.252 -106.951 -4.19415 -27.7583 140 -16.9966 -9.03369 -111.73 -116.714 -76.193 -57.7095 266.18 3.0727 -21.2682))
(setq *av2* #f(49.9331  56.1185 28.9347 130.252 -106.951 -4.19415 -27.7583 140 -29.1714 -9.19364 -117.398 -118.456 -81.5623 -50.4353 263.348 3.0727 -21.2682))
(setq *av3* #f(49.9331  56.1185 28.9347 130.252 -106.951 -4.19415 -27.7583 140 -29.0716 -9.25665 -86.0776 -118.804 -76.2328 -66.8658 270.291 3.0727 -21.2682))

(setq *av-r-approach* #f(49.9331 56.1185 28.9347 130.252 -106.951 -4.19415 -27.7583 140.0 -32.5695 -12.6491 -76.4196 -115.819 -67.3766 -74.5905 266.454 3.0727 -21.2682))
(setq *av-newbowl* #f(49.9125 56.1185 28.9347 130.252 -106.951 -4.19415 -27.7583 145.49 -44.3293 -11.7819 -87.1801 -121.541 -68.7655 -64.4132 266.2 3.3307 -21.2682))

(setq *av1-wall* #f(49.9262 56.1185 28.9347 130.252 -106.951 -4.19415 -27.7583 140.0 -24.8535 -9.11124 -83.3397 -121.542 -94.4255 -78.2682 350.973 1.7647 -21.2682))
(setq *av2-wall* #f(49.9262 56.1185 28.9347 130.252 -106.951 -4.19415 -27.7583 140.0 -24.8535 -9.11124 -83.3397 -121.542 -94.4255 -78.2682 350.973 1.7647 -21.2682))
(setq *av3-wall* #f(49.9262 56.1185 28.9347 130.252 -106.951 -4.19415 -27.7583 140.0 -24.8535 -9.11124 -83.3397 -121.542 -94.4255 -78.2682 350.973 1.7647 -21.2682))

;;start position
(send *pr2* :angle-vector *av-newbowl*)
(send *ri* :angle-vector (send *pr2* :angle-vector) 1000 :larm-controller)
(send *ri* :wait-interpolation)
;;hand the robot the bowl
;;(unix:sleep 2)
(send *pr2* :start-grasp :larm)
(send *ri* :start-grasp :larm :gain 0.05)
(send *ri* :wait-interpolation)

;;one vector inbetween the start to avoid hitting the bowl with the spatula
(send *pr2* :angle-vector *av-r-approach*)
(send *ri* :angle-vector (send *pr2* :angle-vector) 1500 :rarm-controller)
(send *ri* :wait-interpolation)

(setq *l-tcp* (send (send *pr2* :larm :end-coords) :copy-worldcoords))
(setq *ltargetframe* (send *pr2* :larm :end-coords))

;;adjust the bowl for the best starting position
;;was  (0 -10 10)
(send *l-tcp* :translate (float-vector 20 -20 -20))
(send *pr2* :larm :inverse-kinematics *l-tcp*)
(send *ri* :angle-vector (send *pr2* :angle-vector) 700 :larm-controller)
(send *ri* :wait-interpolation)


;;scraping movement of left arm to scrape the bottom of the bowl
(defun scrape-bottom (speed)
  (send *pr2* :angle-vector *av1*)
  (unless (arm-collision-check)
    (send *ri* :angle-vector (send *pr2* :angle-vector) speed :rarm-controller)
    (send *ri* :wait-interpolation)
    )
  ;;publish that the scraping starts
  (setq msg (instance std_msgs::bool :init))
  (send msg :data t)
  (ros::ros-info "msg [~A]" (send msg :data))
  (ros::publish "start_scraping" msg)
  
  (send *pr2* :angle-vector *av2*)
  (unless (arm-collision-check)
    (send *ri* :angle-vector (send *pr2* :angle-vector) speed :rarm-controller)
    (send *ri* :wait-interpolation)
    )
  
  ;;publish that the scraping stopped
  (setq msg (instance std_msgs::bool :init))
  (send msg :data nil)
  (ros::ros-info "msg [~A]" (send msg :data))
  (ros::publish "start_scraping" msg)

  (send *pr2* :angle-vector *av3*)
  (unless (arm-collision-check)
    (send *ri* :angle-vector (send *pr2* :angle-vector) speed :rarm-controller)
    (send *ri* :wait-interpolation)
    )
  )


;;scraping movement of the right arm to scrape the wall of the bowl
(defun scrape-wall (speed)
  (send *pr2* :angle-vector *av1*)
  (unless (arm-collision-check)
    (send *ri* :angle-vector (send *pr2* :angle-vector) speed :rarm-controller)
    (send *ri* :wait-interpolation)
    )
  ;;publish that the scraping starts
  (setq msg (instance std_msgs::bool :init))
  (send msg :data t)
  (ros::ros-info "msg [~A]" (send msg :data))
  (ros::publish "start_scraping" msg)
  
  (send *pr2* :angle-vector *av2*)
  (unless (arm-collision-check)
    (send *ri* :angle-vector (send *pr2* :angle-vector) speed :rarm-controller)
    (send *ri* :wait-interpolation)
    )
  
  ;;publish that the scraping stopped
  (setq msg (instance std_msgs::bool :init))
  (send msg :data nil)
  (ros::ros-info "msg [~A]" (send msg :data))
  (ros::publish "start_scraping" msg)

  (send *pr2* :angle-vector *av3*)
  (unless (arm-collision-check)
    (send *ri* :angle-vector (send *pr2* :angle-vector) speed :rarm-controller)
    (send *ri* :wait-interpolation)
    )
  )


;;EXAMPLE
;;(send *pr2* :reset-pose)
;;(setq collision1 (arm-collision-check)) ;; nil is returned              
;;(send *pr2* :larm :shoulder-r :joint-angle 200)
;;(setq collision2 (arm-collision-check)) ;; some collision check pairs are returned
;;(if collision2
;;    (print "collision 2"))
;;(send *pr2* :reset-pose)


;;predefined scraping pattern to scrape the bowl
;;scrape the bottom of the bowl
(dotimes (i 1)
  (scrape-bottom 1000)
  (send *ri* :spin-once)
  (while (send *ri* :interpolatingp :rarm-controller)
    (send *ri* :spin-once)
    (ros::spin-once);;to see if chatter published 
    (if (send *ri* :simulation-modep)
      (send *ri* :robot-interface-simulation-callback))
    )
  )


;;move the bowl in circle with left hand seems possible to go until 90 degrees if you do it in small steps
;;in - direction collision at the 3rd movement so at 3/10 * PI
(setq omega (/ pi 10))
(setq r 97.5)
(dotimes (i 5)
  ;;translate to the middle of the circle
  (send *l-tcp* :translate (float-vector r 0 0))
  (send *l-tcp* :translate (float-vector (- 0 (* r (cos omega))) (- 0 (* r (sin omega))) 0) )
  (send *l-tcp* :rotate omega :z)
  (send *pr2* :larm :inverse-kinematics *l-tcp*)
  (if (boundp '*irtviewer*)
      (send *irtviewer* :draw-objects))
  (unless (arm-collision-check)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000 :larm-controller)
    (send *ri* :wait-interpolation)
    )
  (when (arm-collision-check)
      (print "collision")
      (print i)
      (print (arm-collision-check)))

  ;;scrape the wall
  (dotimes (i 1)
    (scrape-wall 1000)
    (send *ri* :spin-once)
    (while (send *ri* :interpolatingp :rarm-controller)
      (send *ri* :spin-once)
      (ros::spin-once);;to see if chatter published
      (if (send *ri* :simulation-modep) 
	  (send *ri* :robot-interface-simulation-callback))
      )
    )
  
  )


