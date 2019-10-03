#!/usr/bin/env roseus
;;;;
;;;; make PR2 use a spatula to scrape through a bowl without feedback, this is only a simple experiment in order to gathe data for analysis of PR2 effort accuracy
;;;;

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
;;(unix:sleep 5)		     
;;some time to place the bowl and then close the left gripper
(send *pr2* :start-grasp :larm)
(send *ri* :start-grasp :larm :gain 0.05)
(send *ri* :wait-interpolation)
;;(unix:sleep 5)
;;some time to place the spatula, then close the right gripper
(send *pr2* :start-grasp :rarm)
(send *ri* :start-grasp :rarm :gain 0.05)
(send *ri* :wait-interpolation)

;;add code to move the robot into good start position -> use send *ri* :state :potentio-vector
(setq *av1* #f(49.9331 62.6833 33.1418 127.946 -117.444 -7.41574 -5.72958 51.833 -16.9966 -9.03369 -111.73 -116.714 -76.193 -57.7095 266.18 3.0727 -21.2682))

(send *pr2* :angle-vector *av1*)
(send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
(send *ri* :wait-interpolation)


;;;;;;;;;;;;;;;;;;;;;;;
;;make circle movements
;;;;;;;;;;;;;;;;;;;;;;;

;;initialize variables
(setq *r-tcp* (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
(setq *l-tcp* (send (send *pr2* :larm :end-coords) :copy-worldcoords))
(setq rlinklist (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent)))
(setq llinklist (send *pr2* :link-list (send *pr2* :larm :end-coords :parent)))
(setq rtargetframe (send *pr2* :rarm :end-coords))
(setq ltargetframe (send *pr2* :larm :end-coords))
(objects *r-tcp*)
(objects *l-tcp*) 


;;move a little more front for inverse kinematics
(send *r-tcp* :translate (float-vector 0 0 100))
(send *l-tcp* :translate (float-vector 0 0 -100))
(send *pr2* :inverse-kinematics
      (list *r-tcp* *l-tcp*) ;; specify the goals
      :link-list (list rlinklist llinklist) ;; specify the link chains to be moved?
      :move-target (list rtargetframe ltargetframe) ;; specify which frames on the robot to move to the goals
      )
(send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
(send *ri* :wait-interpolation)

;;use for loop to calculate inverse kinematics step by step
;;safe angle-vectors of inverse kinematics in list
(setq *joint-angle-list* nil)
(setq *joint-angle-time* nil)

(let (r)
  (setq r 80) ;;set the radius to 80mm
  (dotimes (i 80)
  (let (omega)
    (setq omega (* i (/ pi 40)))
    (send *r-tcp* :translate (float-vector 0 (* r (cos omega)) (* r (sin omega)) ) )
    (send *pr2* :inverse-kinematics
      (list *r-tcp* *l-tcp*) ;; specify the goals
      :link-list (list rlinklist llinklist) ;; specify the link chains to be moved?
      :move-target (list rtargetframe ltargetframe) ;; specify which frames on the robot to move to the goals
      )
    (setq potentio-v (send *pr2* :angle-vector))
    (push potentio-v *joint-angle-list*)
    (push 50 *joint-angle-time*) ;;50ms for movement of one step
    (send *r-tcp* :translate (float-vector 0 (- 0 (* r (cos omega))) (- 0 (* r (sin omega)))))
    ;;translate r-tcp back to original position 
  )))
(print "done with joint angle vector calculation")

(send *ri* :angle-vector-sequence *joint-angle-list* *joint-angle-time*)
