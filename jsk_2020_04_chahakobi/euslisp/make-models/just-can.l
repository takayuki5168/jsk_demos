#!/usr/bin/env roseus

;; load pr2-interface.l
(load "package://pr2eus/pr2.l")

;; init 
(pr2)

;; reset-manip-pose
(send *pr2* :reset-manip-pose)

;; init
(load "models/iemon-object.l")
(setq *iemon* (iemon))
(send *iemon* :locate #f(750 0 800) :world)
;; (send *iemon* :rotate (deg2rad 180) :z) 

(load "models/arrow-object.l")
(setq *axis* (arrow))
(send *axis* :move-to (send *iemon* :copy-worldcoords) :world)
(objects (list *pr2* *iemon* *axis*))

;; IKを解いてつかむ

(send *axis* :move-to (send *iemon* :worldcoords) :world)
(send *axis* :translate #f(-80 0 0) :world)

(defun grasp ()
  (send *irtviewer* :draw-objects)

  (send *pr2* :larm :inverse-kinematics
	*axis*
	:rotation-axis :t
	:debug-view t)

  (send *axis* :translate #f(100 0 0) :world)
  (send *irtviewer* :draw-objects)

  (send *pr2* :larm :inverse-kinematics
	*axis*
	:rotation-axis :t
	:debug-view t)

  (send (send *pr2* :larm :end-coords :parent) :assoc *iemon*)

  (send *pr2* :larm :move-end-pos #f(0 0 100) :world
	:debug-view t :look-at-target t)
  )


(defun publish-semantic-annotation-merge (annotation-str)
  (setq msg (instance std_msgs::header :init))
  (send msg :stamp (ros::time-now))
  (send msg :frame_id annotation-str)
  (ros::ros-info "msg [~A]" (send msg :frame_id))
  (ros::publish "semantic_annotation_merge" msg)
  )


(defun pose1 ()
  (send *axis* :newcoords (make-coords :pos #f(400 0 800) :rpy #f(-1.57 0 0)))
  (send *pr2* :larm :inverse-kinematics
	*axis*
	:rotation-axis t
	:debug-view t)
  (send *pr2* :head :look-at (send *iemon* :worldpos))
  (setq *pose1* (send *pr2* :angle-vector))
  )

(defun pose2 ()
  (send *axis* :rotate (deg2rad 80) :z)
  (send *pr2* :larm :inverse-kinematics
	*axis*
	:rotation-axis t
	:debug-view t)
  (send *pr2* :head :look-at (send *iemon* :worldpos))
  (setq *pose2* (send *pr2* :angle-vector))
  )

