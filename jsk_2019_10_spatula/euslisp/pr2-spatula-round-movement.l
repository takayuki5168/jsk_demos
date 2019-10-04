#!/usr/bin/env roseus
;;;;
;;;; make PR2 use a spatula to scrape through a bowl without feedback, this is only a simple experiment in order to gathe data for analysis of PR2 effort accuracy, this time the scraping movement is in circles
;;;;

;;parameters to specify
(setq *n-same-circle* 1) ;;specifies how many times the robot should wipe in same circle
(setq *r-list* '((0 80)(20 120))) ;;(movement left hand, radius)
(setq *n-sample-100* 80) ;;samples/(2*Pi*100), samples for circle with 100mm radius
(setq *t-move* 80) ;;time to move from one pos to next when movign in circle


(load "package://pr2eus/pr2-interface.l")
(ros::roseus "pr2_send_joints")

(setq *ri* (instance pr2-interface :init))
(pr2-init t)

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

;; hand the robot the spatula and bowl
(send *pr2* :stop-grasp)
(send *ri* :stop-grasp)
(send *ri* :wait-interpolation)
(unix:sleep 5)		     
;;some time to place the bowl and then close the left gripper
(send *pr2* :start-grasp :larm)
(send *ri* :start-grasp :larm :gain 0.05)
(send *ri* :wait-interpolation)
(unix:sleep 5)
;;some time to place the spatula, then close the right gripper
(send *pr2* :start-grasp :rarm)
(send *ri* :start-grasp :rarm :gain 0.05)
(send *ri* :wait-interpolation)


;;add code to move the robot into good start position -> use send *ri* :state :potentio-vector

;;START-POSITION
(setq *av1* #f(49.9331 62.6833 33.1418 127.946 -117.444 -7.41574 -5.72958 51.833 -16.9966 -9.03369 -111.73 -116.714 -76.193 -57.7095 266.18 3.0727 -21.2682))

(send *pr2* :angle-vector *av1*)
(send *pr2* :torso :waist-z :joint-angle 250)
(send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
(send *ri* :wait-interpolation)

;;START-POSITION
;;move right tcp dependend on left tcp
(setq *r-tcp* (send (send *pr2* :larm :end-coords) :copy-worldcoords))
(send *r-tcp* :translate (float-vector 130 0 0))
(send *r-tcp* :translate (float-vector 0 150 0))
(send *r-tcp* :rotate (/ pi 2) :z)
(send *r-tcp* :rotate pi :y)

(send *pr2* :inverse-kinematics
	*r-tcp*
	:rotation-axis t
	:move-target (send *pr2* :rarm :end-coords)
	:use-torso nil)
(send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
(send *ri* :wait-interpolation)

;;;;;;;;;;;;;;;;;;;;;;;
;;make circle movements
;;;;;;;;;;;;;;;;;;;;;;;

;;initialize variables
;;(setq *r-tcp* (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
(setq *l-tcp* (send (send *pr2* :larm :end-coords) :copy-worldcoords))
(setq rlinklist (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent)))
(setq llinklist (send *pr2* :link-list (send *pr2* :larm :end-coords :parent)))
(setq rtargetframe (send *pr2* :rarm :end-coords))
(setq ltargetframe (send *pr2* :larm :end-coords))
(objects *r-tcp*)
(objects *l-tcp*) 

;;START-POSITION
;;move a little more front for inverse kinematics
(send *r-tcp* :translate (float-vector 0 0 100))
(send *l-tcp* :translate (float-vector 0 0 -100))
(send *pr2* :inverse-kinematics
      (list *r-tcp* *l-tcp*) ;; specify the goals
      ;;:link-list (list rlinklist llinklist) ;; specify the link chains to be moved?
      :move-target (list rtargetframe ltargetframe) ;; specify which frames on the robot to move to the goals
      :use-torso nil
      )
(send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
(send *ri* :wait-interpolation)


;;use for loop to calculate inverse kinematics step by step
;;safe angle-vectors of inverse kinematics in list
(setq *joint-angle-list* nil)
(setq *joint-angle-time* nil)

;;move in circle with radius in *r-list*
(dolist (dr *r-list*)
  (let (r)
  (let (d)
  ;;!!change to local!!
  (setq d (elt dr 0))
  (setq r (elt dr 1))
  (setq *n-sample* (* (/ *n-sample-100* 100.0) r))
  (let (j-list)
  (let (t-list)
    (setq j-list nil)
    (setq t-list nil)
    ;;one circle with *n-sample* steps
    (send *l-tcp* :translate (float-vector d 0 0)) ;;move left hand to move the bowl
    (dotimes (i *n-sample*)
     (let (omega)
	(setq omega (* 2 (* i (/ pi *n-sample*))))
	(send *r-tcp* :translate (float-vector 0 (* r (cos omega)) (* r (sin omega)) ) )
	(send *r-tcp* :rotate (+ omega 90) :x)
	(send *pr2* :inverse-kinematics 
	      (list *r-tcp* *l-tcp*)
	      ;;:link-list (list rlinklist llinklist)
	      :move-target (list rtargetframe ltargetframe)
	      :use-torso nil
	      )
	(setq potentio-v (send *pr2* :angle-vector))	
	(push potentio-v j-list)
	(push *t-move* t-list)
	;;translate r-tcp back to the center of the bowl
	(send *r-tcp* :rotate (- 0 (+ omega 90)) :x)
	(send *r-tcp* :translate (float-vector 0 (- 0 (* r (cos omega))) (- 0 (* r (sin omega)))))
	))
    ;;move in circle with same radius multiple times
    (dotimes (j *n-same-circle*)
      (setq *joint-angle-time* (append *joint-angle-time* t-list))
      (setq *joint-angle-list* (append *joint-angle-list* j-list))
      )
    (send *l-tcp* :translate (float-vector (- 0 d) 0 0)) ;;move the l-tcp back to start position
  )))))

(print "done with joint angle vector calculation")
(send *ri* :angle-vector-sequence *joint-angle-list* *joint-angle-time*)

