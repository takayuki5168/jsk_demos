#!/usr/bin/env roseus

;; load pr2-interface.l
(load "package://pr2eus/pr2-interface.l")

;; init 
(pr2-init)

;; reset-pose
(send *pr2* :reset-pose)
(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
