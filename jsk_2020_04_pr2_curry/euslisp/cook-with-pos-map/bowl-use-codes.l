(defun bowl-init ()
  ;; init
  (load "move-to-kitchen-with-map.l")
  
  (objects (list *pr2* *scene*))
  (update-view)
  
  ;; set global num
  (setq *s-time* 3)
  
  ;; def bowl grasp coords
  (setq *bowl-1-coords* (make-coords :pos (float-vector 362 8446 30880) :rpy (float-vector 2.766 0.889 1.742)))
  (setq *bowl-2-coords* (make-coords :pos (float-vector 344 8616 30880) :rpy (float-vector 3.016 0.995 1.718)))
  (setq *bowl-2-set-coords* (make-coords :pos (float-vector 584.588 8546.67 30885.165) :rpy (float-vector -2.389 0.788 1.578)))

  (move-to-kitchen-irt) ;;not use real robot
  )

(bowl-init)

;; set codes
(warn "please run (now-set-bowls) or (set-bowl-1) ~%")

;; def utils-funcs of bowl
(defun reset-rarm ()
  (setq *rav0* (float-vector -25.6468 7.87241 -48.2704 -85.3763 253.675 -77.8887 -147.102))
  (send *pr2* :rarm :angle-vector *rav0*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)  
  
  (setq *rav1* (float-vector -20.8048 54.2396 -5.42985 -119.734 197.32 -75.8373 -189.059))
  (send *pr2* :rarm :angle-vector *rav1*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  ;; (setq *rav2* (float-vector -30.0303 1.63624 -169.166 -116.783 100.843 -98.3015 -58.1037))
  ;; (send *pr2* :rarm :angle-vector *rav2*)
  ;; (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  ;; (send *ri* :wait-interpolation)

  (setq *rav3* (float-vector -75.0 50.0 -110.0 -110.0 20.0 -10.0 -10.0))
  (send *pr2* :rarm :angle-vector *rav3*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  )

(defun reset-larm ()
  ;; 左手を少し上に
  (send *arrow* :newcoords (send (send *pr2* :larm :end-coords) :copy-worldcoords))
  (send *arrow* :translate (float-vector 0 0 100) :world)
  (send *pr2* :larm :inverse-kinematics
  	(send *arrow* :copy-worldcoords)
  	:rotation-axis t
  	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  ;; 少し上にして水平に
  (setq *lav1* (float-vector 16.6498 -20.1996 123.812 -51.9785 -42.6397 -65.2851 -0.059116))
  ;; (setq *lav1* (float-vector 16.6222 -20.2483 123.957 -51.9842 -42.7912 -65.3348 -437.304))
  (send *pr2* :larm :angle-vector *lav1*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  ;; 手前にする
  (setq *lav2* (float-vector 41.8129 -20.2467 85.9536 -82.3235 -13.8272 -47.9804 -0.0))
  ;; (setq *lav2* (float-vector 41.8129 -20.2467 85.9536 -82.3235 -13.8272 -47.9804 -458.658))
  (send *pr2* :larm :angle-vector *lav2*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  
  (setq *lav3* (float-vector 75.8217 52.0712 108.49 -109.796 -2.86542 -9.47056 -0.0))
  ;; (setq *lav3* (float-vector 75.8217 52.0712 108.49 -109.796 -2.86542 -9.47056 -401.362))
  (send *pr2* :larm :angle-vector *lav3*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 6000)
  (send *ri* :wait-interpolation)  

  (setq *lav4* (float-vector 75.0 50.0 110.0 -110.0 -20.0 -10.0 -10.0))
  (send *pr2* :larm :angle-vector *lav4*)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)  

  )


(defun before-recieve-larm ()
  ;;少し上に
  (send *arrow* :newcoords (send (send *pr2* :larm :end-coords) :copy-worldcoords))
  (send *arrow* :translate #f(0 0 250) :world)
  (send *pr2* :larm :inverse-kinematics
	(send *arrow* :copy-worldcoords)
	:rotation-axis nil
	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  ;;少し前に
  (send *arrow* :newcoords (make-coords :pos (send (send (send *pr2* :larm :end-coords) :copy-worldcoords) :worldpos)))
  (send *arrow* :translate #f(-200 50 50) :world)
  (send *arrow* :rotate (deg2rad 90) :z :world)
  (send *arrow* :rotate (deg2rad 90) :x)
  (send *arrow* :rotate (deg2rad -60) :x :world)  
  (send *pr2* :larm :inverse-kinematics
  	(send *arrow* :copy-worldcoords)
  	:rotation-axis t
  	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  
  )

(defun put-bowl-larm ()
  ;; (send *arrow* :translate (float-vector 0 0 200) :world) ;; がしてある状態で使う
  (send *pr2* :larm :inverse-kinematics
  	(send *arrow* :copy-worldcoords)
  	:rotation-axis t
  	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  
  (send *arrow* :translate (float-vector 0 0 -100) :world)
  (send *pr2* :larm :inverse-kinematics
  	(send *arrow* :copy-worldcoords)
  	:rotation-axis t
  	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (send *arrow* :translate (float-vector 0 0 -100) :world)
  (send *pr2* :larm :inverse-kinematics
  	(send *arrow* :copy-worldcoords)
  	:rotation-axis t
  	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  (send *ri* :stop-grasp :larm :wait t)

  (send *arrow* :translate (float-vector 0 0 100) :world)
  (send *pr2* :larm :inverse-kinematics
  	(send *arrow* :copy-worldcoords)
  	:rotation-axis t
  	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (send *arrow* :translate (float-vector 0 0 100) :world)
  (send *pr2* :larm :inverse-kinematics
  	(send *arrow* :copy-worldcoords)
  	:rotation-axis t
  	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  )


(defun before-recieve-rarm ()
  ;;少し上に
  (send *arrow* :newcoords (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
  (send *arrow* :translate (float-vector 0 0 250) :world)
  (send *pr2* :rarm :inverse-kinematics
	(send *arrow* :copy-worldcoords)
	:rotation-axis nil
	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  ;;少し前に
  (send *arrow* :newcoords (make-coords :pos (send (send (send *pr2* :rarm :end-coords) :copy-worldcoords) :worldpos)))
  (send *arrow* :translate (float-vector -200 50 50) :world)
  (send *arrow* :rotate (deg2rad -90) :z :world)
  (send *arrow* :rotate (deg2rad 90) :x)
  (send *arrow* :rotate (deg2rad 60) :x :world)  
  (send *pr2* :rarm :inverse-kinematics
  	(send *arrow* :copy-worldcoords)
  	:rotation-axis t
  	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  )

(defun put-bowl-rarm ()
  ;; (send *arrow* :translate (float-vector 0 0 200) :world) ;; がしてある状態で使う
  (send *pr2* :rarm :inverse-kinematics
  	(send *arrow* :copy-worldcoords)
  	:rotation-axis t
  	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  
  (send *arrow* :translate (float-vector 0 0 -100) :world)
  (send *pr2* :rarm :inverse-kinematics
  	(send *arrow* :copy-worldcoords)
  	:rotation-axis t
  	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (send *arrow* :translate (float-vector 0 0 -100) :world)
  (send *pr2* :rarm :inverse-kinematics
  	(send *arrow* :copy-worldcoords)
  	:rotation-axis t
  	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  (send *ri* :stop-grasp :rarm :wait t)

  (send *arrow* :translate (float-vector 0 0 100) :world)
  (send *pr2* :rarm :inverse-kinematics
  	(send *arrow* :copy-worldcoords)
  	:rotation-axis t
  	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (send *arrow* :translate (float-vector 0 0 100) :world)
  (send *pr2* :rarm :inverse-kinematics
  	(send *arrow* :copy-worldcoords)
  	:rotation-axis t
  	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation) 
  )

(defun grasp-bowl-larm ()
  (before-recieve-larm)

  (send *ri* :stop-grasp :larm :wait t)
  
  ;; ボウルの位置
  (send *arrow* :newcoords (send *target* :copy-worldcoords))
  (send *arrow* :translate (float-vector 0 0 200) :world)
  (send *pr2* :larm :inverse-kinematics
  	(send *arrow* :copy-worldcoords)
  	:rotation-axis t
  	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  
  (send *arrow* :translate (float-vector 0 0 -100) :world)
  (send *pr2* :larm :inverse-kinematics
  	(send *arrow* :copy-worldcoords)
  	:rotation-axis t
  	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (send *arrow* :translate (float-vector 0 0 -100) :world)
  (send *pr2* :larm :inverse-kinematics
  	(send *arrow* :copy-worldcoords)
  	:rotation-axis t
  	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  (send *ri* :start-grasp :larm :wait t)

  (send *arrow* :translate (float-vector 0 0 100) :world)
  (send *pr2* :larm :inverse-kinematics
  	(send *arrow* :copy-worldcoords)
  	:rotation-axis t
  	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (send *arrow* :translate (float-vector 0 0 100) :world)
  (send *pr2* :larm :inverse-kinematics
  	(send *arrow* :copy-worldcoords)
  	:rotation-axis t
  	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)  
  )

(defun grasp-bowl-rarm ()
  (before-recieve-rarm)
  
  (send *ri* :stop-grasp :rarm :wait t)
  
  ;; ボウルの位置
  (send *arrow* :newcoords (send *target* :copy-worldcoords))
  (send *arrow* :translate (float-vector 0 0 200) :world)
  (send *pr2* :rarm :inverse-kinematics
  	(send *arrow* :copy-worldcoords)
  	:rotation-axis t
  	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  
  (send *arrow* :translate (float-vector 0 0 -100) :world)
  (send *pr2* :rarm :inverse-kinematics
  	(send *arrow* :copy-worldcoords)
  	:rotation-axis t
  	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (send *arrow* :translate (float-vector 0 0 -100) :world)
  (send *pr2* :rarm :inverse-kinematics
  	(send *arrow* :copy-worldcoords)
  	:rotation-axis t
  	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  (send *ri* :start-grasp :rarm :wait t)

  (send *arrow* :translate (float-vector 0 0 100) :world)
  (send *pr2* :rarm :inverse-kinematics
  	(send *arrow* :copy-worldcoords)
  	:rotation-axis t
  	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (send *arrow* :translate (float-vector 0 0 100) :world)
  (send *pr2* :rarm :inverse-kinematics
  	(send *arrow* :copy-worldcoords)
  	:rotation-axis t
  	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)  
  )


;; funcs
(defun receive-bowl-1 ()
  (before-recieve-larm)
  
  (send *ri* :stop-grasp :larm :wait t)
  (send *ri* :speak-jp "ボウルを握らせて下さい")
  (unix:sleep s-time)
  (send *ri* :start-grasp :larm :wait t)

  (warn "please run (put-bowl-1) ~%")
  )

(defun put-bowl-1 ()
  ;; ボウル１を置く位置
  ;; (send *arrow* :newcoords (make-coords :pos (float-vector 362 1946 880) :rpy (float-vector 2.766 0.889 1.742)))  ;;
  (send *arrow* :newcoords (send *bowl-1-coords* :copy-worldcoords))
  (send *arrow* :translate (float-vector 0 0 200) :world)

  (put-bowl-larm)
  )

(defun recieve-bowl-2 ()
  (before-recieve-rarm)
  
  (send *ri* :stop-grasp :rarm :wait t)
  (send *ri* :speak-jp "ボウルを握らせて下さい")
  (unix:sleep s-time)
  (send *ri* :start-grasp :rarm :wait t)

  (warn "please run (put-bowl-2) ~%")
  )

(defun put-bowl-2 ()
  ;; ボウル２を置く位置
  ;; (send *arrow* :newcoords (make-coords :pos (float-vector 344 2116 880) :rpy (float-vector 3.016 0.995 1.718)))
  (send *arrow* :newcoords (send *bowl-2-coords* :copy-worldcoords))
  (send *arrow* :translate (float-vector 0 0 200) :world)

  (put-bowl-rarm)  
  )

(defun grasp-bowl-2 ()
  ;; ボウル２の位置
  (setq *target* (make-coords))
  ;; (send *target* :newcoords (make-coords :pos (float-vector 344 2116 880) :rpy (float-vector 3.016 0.995 1.718)))
  (send *target* :newcoords (send *bowl-1-coords* :copy-worldcoords))
  
  (grasp-bowl-rarm)
  
  (warn "please run (set-bowl-2) ~%")
  )

(defun set-bowl-2 ()
  (send *pr2* :rarm :move-end-pos (float-vector 100 0 0) :world)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  ;; ボウル２を置く位置 set
  ;; (send *arrow* :newcoords (make-coords :pos (float-vector 584.588 2046.672 885.165) :rpy (float-vector -2.389 0.788 1.578)))
  (send *arrow* :newcoords (send *bowl-2-set-coords* :copy-worldcoords))
  (send *arrow* :translate (float-vector 0 0 200) :world)
  
  (put-bowl-rarm)  
  )


(defun grasp-bowl-2-set ()
  ;; ボウル２の位置 set
  (setq *target* (make-coords))
  ;; (send *target* :newcoords (make-coords :pos (float-vector 550.911 1925.179 885.843) :rpy (float-vector 2.561 1.128 2.104)))
  (send *target* :newcoords (send *bowl-2-set-coords* :copy-worldcoords))
  
  (grasp-bowl-larm)

  (warn "please run (transfer-to-pot) ~%")  
  )

(defun put-bowl-2-set ()
  ;; ボウル２の位置 set
  ;; (send *arrow* :newcoords (make-coords :pos (float-vector 550.911 1925.179 885.843) :rpy (float-vector 2.561 1.128 2.104)))
  (send *arrow* :newcoords (send *bowl-2-set-coords* :copy-worldcoords))
  (send *arrow* :translate (float-vector 0 0 200) :world)

  (put-bowl-larm)
  )

(defun grasp-bowl-1 ()
  ;; ボウル１の位置
  (setq *target* (make-coords))
  ;; (send *target* :newcoords (make-coords :pos (float-vector 362 1946 880) :rpy (float-vector 2.766 0.889 1.742)))  
  (send *target* :newcoords (send *bowl-1-coords* :copy-worldcoords))
  
  (grasp-bowl-larm)

  (warn "please run (transfer-to-pot) ~%")  
  )


;; transfer funcs
(defun transfer-to-pot ()
  (send *arrow* :newcoords (make-coords :pos (float-vector 550.911 8325.179 31085.843) :rpy (float-vector 2.561 1.128 2.104))) ;; for eng2 model
  ;; (send *arrow* :newcoords (make-coords :pos (float-vector 550.911 1825.179 1085.843) :rpy (float-vector 2.561 1.128 2.104)))
  (send *pr2* :larm :inverse-kinematics
	(send *arrow* :copy-worldcoords)
	:rotation-axis t
	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  
  (send *pr2* :larm :move-end-pos (float-vector 0 -100 0) :world)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (send *ri* :wait-interpolation)

  (send *pr2* :larm :move-end-pos (float-vector 0 -100 0) :world)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (send *ri* :wait-interpolation)

  (send *pr2* :larm :move-end-pos (float-vector 0 -100 0) :world)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  
  (setq *larm1* (float-vector 42.6232 6.188 43.7963 -57.5889 111.452 -100.941 -32.9738))
  (send *pr2* :larm :angle-vector *larm1*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  
  (setq *larm2* (float-vector 35.0371 5.51912 50.9811 -60.5584 159.61 -110.561 -25.9414))
  (send *pr2* :larm :angle-vector *larm2*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
  
  (send *arrow* :newcoords (make-coords :pos (float-vector 463.549 1518.534 1099.284) :rpy (float-vector -1.743 0.802 -1.842)))
  (send *pr2* :larm :inverse-kinematics
	(send *arrow* :copy-worldcoords)
	:rotation-axis t
	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  
  (send *arrow* :newcoords (make-coords :pos (float-vector 465 1505 1130) :rpy (float-vector -1.571 0.436 -1.571))) 
  (send *pr2* :larm :inverse-kinematics
	(send *arrow* :copy-worldcoords)
	:rotation-axis t
	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  
  (warn "please run (after-transfer) ~%")  
  )

(defun after-transfer ()
  (setq *larm2* (float-vector 35.0371 5.51912 50.9811 -60.5584 159.61 -110.561 -25.9414))
  (send *pr2* :larm :angle-vector *larm2*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
  (send *ri* :wait-interpolation)

  (setq *larm1* (float-vector 42.6232 6.188 43.7963 -57.5889 111.452 -100.941 -32.9738))
  (send *pr2* :larm :angle-vector *larm1*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  
  (send *pr2* :larm :move-end-pos (float-vector 0 100 0) :world)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (send *ri* :wait-interpolation)

  (send *pr2* :larm :move-end-pos (float-vector 0 100 0) :world)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (send *ri* :wait-interpolation)

  (send *pr2* :larm :move-end-pos (float-vector 0 100 0) :world)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  
  (warn "please run (put-bowl-1) or (put-bowl-2-set) ~%")  
  )





;; now funcs
;; ボウル２をセット
(defun now-set-2 ()
  (recieve-bowl-2)
  (put-bowl-2)
  (reset-rarm)
  )

;; ボウル１をセット
(defun now-set-1 ()
  (recieve-bowl-1)
  (put-bowl-1)
  (reset-larm)
  )

;; ボウル１と２をセット
(defun now-set-all ()
  (now-set-1)
  (now-set-2)
  )

;; ボウル２をまな板上に
(defun switch-2 ()
  (grasp-bowl-2)
  (set-bowl-2)
  (reset-rarm)
  )

(defun transfer-2 ()
  (transfer-pot)
  (after-pot)
  )

(defun now-test ()
  (transfer-pot)
  (after-pot)
  )

;; ボウル２をまな板上に置いて使う
(defun now-devel ()
  (grasp-bowl-2)
  (set-bowl-2)
  (reset-rarm)

  (transfer-pot)
  (after-pot)
  )
