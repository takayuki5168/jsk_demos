#!/usr/bin/env roseus

;; load pr2-interface.l
(load "package://pr2eus/pr2-interface.l")

;; init 
(pr2-init)

;; reset-manip-pose
(send *pr2* :reset-manip-pose)
(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
(send *ri* :wait-interpolation)


;; トピックなどを定義
(defvar *topic-name* "/icp_registration/output_pose")

;; subscribeして認識



;; ikを解く
