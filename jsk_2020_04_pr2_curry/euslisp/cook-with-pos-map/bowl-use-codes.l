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
  
  )

(move-to-kitchen-irt) ;;not use real robot
(bowl-init)

;; set codes
(warn "please run (now-set-bowls) or (set-bowl-1) ~%")

;; def utils-funcs of bowl
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
