(defun water-init ()
  ;; init
  (load "move-to-kitchen-with-map.l")
  
  ;; set global num
  (setq *s-time* 3)

  (move-to-sink-irt)
  )

(water-init)

(warn "If you use real robot, run (update-pos) ~%")
(warn "please run (grasp-lever) ~%")


;; def util-funcs
(defun reset-rarm ()
  (setq *rav0* (float-vector -25.6468 7.87241 -48.2704 -85.3763 253.675 -77.8887 -147.102))
  (send *pr2* :rarm :angle-vector *rav0*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)  
  
  (setq *rav1* (float-vector -20.8048 54.2396 -5.42985 -119.734 197.32 -75.8373 -189.059))
  (send *pr2* :rarm :angle-vector *rav1*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  (setq *rav3* (float-vector -75.0 50.0 -110.0 -110.0 20.0 -10.0 -10.0))
  (send *pr2* :rarm :angle-vector *rav3*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  )

;; water funcs
(defun grasp-lever ()
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

  ;;グリッパを微妙に開く
  (send *ri* :move-gripper :rarm 0.05)

  ;; 水道のレバーの位置
  (send *arrow* :newcoords (send *fauset-lever-rarm-coords* :copy-worldcoords))
  
  (send *arrow* :translate #f(100 60 20) :world)
  (send *pr2* :larm :inverse-kinematics
	(send *arrow* :copy-worldcoords)
	:rotation-axis t
	:debug-view t)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  (send *arrow* :translate #f(-100 -60 -20) :world)
  (send *pr2* :larm :inverse-kinematics
	(send *arrow* :copy-worldcoords)
	:rotation-axis t
	:debug-view t)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  (send *ri* :start-grasp :rarm :wait t)
  
  (warn "please run (water-on) ~%") 
  )


(defun water-on ()
  ;; レバーを掴んでいる状態から
  (send *arrow* :newcoords (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
  (send *arrow* :translate (float-vector 0 0 80) :world)
  (send *pr2* :rarm :inverse-kinematics
	(send *arrow* :copy-worldcoords)
	:rotation-axis nil
	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  
  (warn "please run (water-off) ~%") 
  )


(defun water-off ()
  (send *arrow* :newcoords (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
  (send *arrow* :translate (float-vector 0 0 -80) :world)
  (send *pr2* :rarm :inverse-kinematics
	(send *arrow* :copy-worldcoords)
	:rotation-axis nil
	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  (warn "please run (release-lever)")
  )


(defun release-lever ()
  ;;グリッパを微妙に開く
  (send *ri* :move-gripper :rarm 0.05)
  
  (send *arrow* :newcoords (send *fauset-lever-rarm-coords* :copy-worldcoords))
  (send *arrow* :translate #f(100 60 20) :world)
  (send *pr2* :larm :inverse-kinematics
	(send *arrow* :copy-worldcoords)
	:rotation-axis t
	:debug-view t)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  (reset-rarm)
  )
