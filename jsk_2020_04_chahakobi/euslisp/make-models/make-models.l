#!/usr/bin/env roseus

;; load pr2-interface.l
(load "package://pr2eus/pr2-interface.l")

;; init 
(pr2-init)


;; ros init
(ros::roseus "pr2_send_joints")
(ros::advertise "semantic_annotation_merge" std_msgs::header 1)


;; reset-manip-pose
(send *pr2* :reset-manip-pose)
(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
(send *ri* :wait-interpolation)


;; init
(load "models/kettle-object.l")
(setq *kettle* (kettle))
(send *kettle* :locate #f(780 0 740) :world)
(send *kettle* :rotate (deg2rad 180) :z) 

(load "models/arrow-object.l")
(setq *axis* (arrow))
(send *axis* :move-to (send *kettle* :handle-handle0) :world)
(objects (list *pr2* *kettle* *axis*))

;; IKを解いてつかむ

(send *axis* :move-to (send (send *kettle* :handle-handle0) :worldcoords) :world)
(send *axis* :translate #f(-80 0 0) :world)
(send *axis* :rotate (deg2rad 180) :y) 
(send *axis* :rotate (deg2rad 180) :x)
(send *irtviewer* :draw-objects)

(send *ri* :stop-grasp :larm)
(send *pr2* :larm :inverse-kinematics
      *axis*
      :rotation-axis :t
      :debug-view t)
(send *ri* :angle-vector (send *pr2* :angle-vector))
(send *ri* :wait-interpolation)

(send *axis* :translate #f(100 0 0) :world)
(send *irtviewer* :draw-objects)

(send *pr2* :larm :inverse-kinematics
      *axis*
      :rotation-axis :t
      :debug-view t)
(send *ri* :angle-vector (send *pr2* :angle-vector))
(send *ri* :wait-interpolation)

(send *ri* :start-grasp :larm)
(send (send *pr2* :larm :end-coords :parent) :assoc *kettle*)


(defun publish-semantic-annotation-merge (annotation-str)
  (setq msg (instance std_msgs::header :init))
  (send msg :stamp (ros::time-now))
  (send msg :frame_id annotation-str)
  (ros::ros-info "msg [~A]" (send msg :frame_id))
  (ros::publish "semantic_annotation_merge" msg)
  )


(defun pose1 ()
  (send *axis* :newcoords (make-coords :pos #f(400 0 800) :rpy #f(-1.57 0 -1.57)))
  (send *pr2* :larm :inverse-kinematics
	*axis*
	;; (make-coords :pos #f(358.676 83.313 803.969) :rpy #f(0 0 0))
	;; (make-coords :pos #f(358.676 83.313 803.969) :rpy #f(-1.562 0.04 3.113))
	:rotation-axis t
	:debug-view t)
  (setq *pose1* (send *pr2* :angle-vector))
  (send *ri* :angle-vector *pose1*)
  (send *ri* :wait-interpolation)
  (publish-semantic-annotation-merge (format nil "vision_0"))
  (unix:sleep 1)
  )

(defun pose2 ()
  (send *axis* :rotate (deg2rad 80) :y)
  (send *pr2* :larm :inverse-kinematics
	*axis*
	;; (make-coords :pos #f(358.676 83.313 803.969) :rpy #f(0 0 0))
	;; (make-coords :pos #f(358.676 83.313 803.969) :rpy #f(-1.562 0.04 3.113))
	:rotation-axis t
	:debug-view t)
  (setq *pose2* (send *pr2* :angle-vector))
  (send *ri* :angle-vector *pose2*)
  (send *ri* :wait-interpolation)
  (publish-semantic-annotation-merge (format nil "vision_2"))
  (unix:sleep 1)
  (publish-semantic-annotation-merge (format nil "vision_end"))
  )

