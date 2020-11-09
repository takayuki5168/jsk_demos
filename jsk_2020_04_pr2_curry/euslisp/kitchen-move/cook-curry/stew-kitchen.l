1;5202;0c(defun init ()
  (if (not (boundp '*room*))
      (progn
        (load "../../model/my-room73b2-scene.l")
        (setq *room* (room73b2))
        ))
    
  (if (not (boundp '*kitchen*))
      (progn 
        (load "models/room73b2-kitchen-object.l")
        (setq *kitchen* (room73b2-kitchen))
        (send *kitchen* :transform (make-coords :pos (float-vector 305.0 2175.0 20.0) :rot #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0))))
        ))

  (if (not (boundp '*pr2*))
      (progn
        (load "package://pr2eus/pr2-interface.l")
        (pr2-init)
        ))

  ;; set まな板
  (load "../../model/cutting-board.l")
  (setq *cutting-board* (cutting-board))
  (send *cutting-board* :transform (make-coords :pos (float-vector 500.0 2000.0 840.0) :rot #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0))))

  ;; set objects
  (load "../../model/peeler.l")
  (load "../../model/potato-peeler.l")
  (load "../../model/knife-new-attention.l")
  (load "models/arrow-object.l")
  (setq *arrow* (arrow))

  ;; set spots
  (setq *cook-spot* (send *room73b2* :spot "cook-spot"))
  (setq *sink-spot* (make-coords :pos #f(1055 2600 0) :rpy #f(3.14 0 0))) 
  (setq *kitchen-spot* (send *room73b2* :spot "kitchen-spot"))
  (setq *stove-spot* (make-coords :pos #f(1400 1600 0) :rpy #f(3.14 0 0)))
  

  ;; キッチンの場所まで移動 (初期値)
  (defun move-to-kitchen-irt ()
    (send *pr2* :move-to *kitchen-spot* :world)
    )
  (move-to-kitchen-irt)
  
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))

  ;; def global values
  (setq s-time 3)
  
  (setq adj 0)

  ;; set ladle
  (load "../../model/ladle.l")
  (setq *ladle* (ladle))

  ;; set pot
  (load "../../model/cooking-pot-73b2.l")
  (setq *pot* (cooking-pot))
  (send *pot* :newcoords (make-coords :pos (float-vector 420.0 1500.0 840.0) :rot #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0))))
  
  (setq width 90)
  (setq length 60)
  (setq height 60)
  )


(defun set-pose ()
  (send *pr2* :reset-manip-pose)
  (send *pr2* :torso_lift_joint :joint-angle 325)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  (warn "please run (on-stove) ~%")
  )

(init)

(objects (list *pr2* *pot* *kitchen*))


(warn "please run (set-ladle) ~%")

(defun set-ladle ()
  ;; おたまをセットする．
  (send *ladle* :newcoords (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
  (send *ladle* :rotate (deg2rad 90) :x)
  (send *pr2* :rarm :end-coords :assoc *ladle*)

  (objects (list *kitchen* *pr2* *ladle* *pot*))
  (send *irtviewer* :look-all
	(geo::make-bounding-box
	 (flatten (send-all (send *pr2* :bodies) :vertices))))  

  (warn "please run (peel-test) ~%")
  )

(defun recieve-ladle ()
  (setq *rav1* (float-vector -20.8048 54.2396 -5.42985 -119.734 197.32 -75.8373 -189.059))
  (send *pr2* :rarm :angle-vector *rav1*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  
  (send *arrow* :newcoords (make-coords :pos (float-vector 540.574 1989.496 1127.996) :rpy (float-vector 3.125 -0.122 -0.004)))
  (send *pr2* :rarm :inverse-kinematics
	(send *arrow* :copy-worldcoords)
	:rotation-axis t
	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  
  (send *ri* :stop-grasp :rarm :wait t)  
  (send *ri* :speak-jp "おたまを握らせて下さい")
  (unix:sleep s-time)
  (send *ri* :start-grasp :rarm :wait t)
  )

(defun put-ladle ()
  (setq *rav1* #f(27.356 0.296651 -2.77311 -37.6649 179.549 -113.236 -246.974))
  (send *pr2* :rarm :angle-vector *rav1*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  ;; 置く位置
  (send *arrow* :newcoords (make-coords :pos (float-vector 399.539 1874.817 1035.216) :rpy (float-vector -2.722 1.036 -1.177)))
  (send *arrow* :translate #f(60 0 100) :world)
  (send *pr2* :rarm :inverse-kinematics
	(send *arrow* :copy-worldcoords)
	:rotation-axis t
	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  
  (send *arrow* :translate #f(-60 0 -100) :world)
  (send *pr2* :rarm :inverse-kinematics
	(send *arrow* :copy-worldcoords)
	:rotation-axis t
	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  ;; はなす
  (send *ri* :stop-grasp :rarm :wait t)
  (send (send *pr2* :rarm :end-coords) :dissoc *ladle*)

  ;; 手を上げる
  (send *arrow* :newcoords (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
  (send *arrow* :translate #f(0 0 150) :world)
  (send *pr2* :rarm :inverse-kinematics
	(send *arrow* :copy-worldcoords)
	:rotation-axis t
	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  (setq *rav1* #f(27.356 0.296651 -2.77311 -37.6649 179.549 -113.236 -246.974))
  (send *pr2* :rarm :angle-vector *rav1*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (send *arrow* :newcoords (make-coords :pos (float-vector 540.574 1989.496 1127.996) :rpy (float-vector 3.125 -0.122 -0.004)))
  (send *pr2* :rarm :inverse-kinematics
	(send *arrow* :copy-worldcoords)
	:rotation-axis t
	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  (setq *rav0* #f(-20.8048 54.2396 -5.42985 -119.734 197.32 -75.8373 -189.059))
  (send *pr2* :rarm :angle-vector *rav0*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  )

(defun grasp-ladle ()
  (setq *rav1* #f(27.356 0.296651 -2.77311 -37.6649 179.549 -113.236 -246.974))
  (send *pr2* :rarm :angle-vector *rav1*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  
  ;; 掴む位置
  (send *arrow* :newcoords (make-coords :pos (float-vector 399.539 1874.817 1035.216) :rpy (float-vector -2.722 1.036 -1.177)))
  
  (send *ri* :stop-grasp :rarm :wait t)
  
  (send *arrow* :translate #f(60 0 100) :world)
  (send *pr2* :rarm :inverse-kinematics
	(send *arrow* :copy-worldcoords)
	:rotation-axis t
	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  
  (send *arrow* :translate #f(-60 0 -100) :world)
  (send *pr2* :rarm :inverse-kinematics
	(send *arrow* :copy-worldcoords)
	:rotation-axis t
	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  
  ;; つかむ
  (send *ri* :start-grasp :rarm :wait t)
  (send *ladle* :newcoords (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
  (send *ladle* :rotate (deg2rad 90) :x)
  (send (send *pr2* :rarm :end-coords) :assoc *ladle*)
  
  ;; 手を上げる
  (send *arrow* :newcoords (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
  (send *arrow* :translate #f(0 0 150) :world)
  (send *pr2* :rarm :inverse-kinematics
	(send *arrow* :copy-worldcoords)
	:rotation-axis t
	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  (setq *rav1*   #f(27.356 0.296651 -2.77311 -37.6649 179.549 -113.236 -246.974))
  (send *pr2* :rarm :angle-vector *rav1*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  )


(defun now-set-ladle ()
  (recieve-ladle)
  (put-ladle)
  )
