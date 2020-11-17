(defun ih-init ()
  ;; init
  (load "move-to-kitchen-with-map.l")
  (load "tool-use-codes.l")

  ;; set pot
  (load "package://jsk_2020_04_pr2_curry/euslisp/model/cooking-pot-73b2.l")
  (setq *pot* (cooking-pot))
  (send *pot* :newcoords (send *pot-coords* :copy-worldcoords))
  

  ;; def global coords
  (setq *pot-larm-from-above* (make-coords :pos (float-vector 639 8109 30971) :rpy (float-vector -3.069 1.306 -0.087)))
  (setq *pot-larm-from-below* (make-coords :pos (float-vector 644 8104 30947) :rpy (float-vector 3.125 -0.909 3.119)))
  
  ;; set global num
  (setq *s-time* 3)

  (move-to-kitchen-irt)
  (objects (list *pr2* *scene* *spatula* *ladle* *pot*))
  (update-view)
  )

(ih-init)


(defun set-pose-3 ()
  (send *pr2* :reset-manip-pose)
  (send *pr2* :torso_lift_joint :joint-angle 325)
  (send *pr2* :larm :angle-vector #f(117.007 48.3805 113.439 -103.326 38.7107 -5.72958 14.4628))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)  
  )

(set-pose-3)

(warn "If you use real robot, run (update-pos) ~%")


;; add for sense
(setq *x-thre* -15)

(ros::roseus "stove_rec_test" :anonymous t)
(setq *topic-name* "/left_endeffector/wrench")
(ros::load-ros-manifest "geometry_msgs")
(setq *cb-flag* nil)
(defun wrench-x-cb (msg)
  (when msg
    (let ((x nil))
      (setq x (send msg :wrench :force :x))
      (if (< x *x-thre*)
          (progn 
            (setq *cb-flag* t)
            (format t "OK!! x: ~A ~%" x)
            )
        (format t "x: ~A ~%" x))
      )))


;; def util-funcs
(defun reset-rarm ()
  (setq *rav0* #f(-20.8048 54.2396 -5.42985 -119.734 197.32 -75.8373 -189.059))
  (send *pr2* :rarm :angle-vector *rav0*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (setq *rav2* #f(-30.0303 1.63624 -169.166 -116.783 100.843 -98.3015 -58.1037))
  (send *pr2* :rarm :angle-vector *rav2*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  
  (setq *rav3* #f(-75.0 50.0 -110.0 -110.0 20.0 -10.0 -10.0))
  (send *pr2* :rarm :angle-vector *rav3*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  )

(defun reset-larm-3 ()
  (setq *larm0* #f(117.007 48.3805 113.439 -103.326 38.7107 -5.72958 14.4628))
  (send *pr2* :larm :angle-vector *larm0*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)  
  )


;; IH funcs 
(defun push-stove-button ()
  ;; (send *arrow* :translate #f(130 0 0) :world) ;;をしている前提で始める
  
  (setq avs (list))
  (setq tms (list))
  
  (dotimes (i 15)
    (send *pr2* :larm :move-end-pos #f(-10 0 0) :world)
    (setq av (send *pr2* :angle-vector))
    (setq avs (append avs (list av)))
    (setq tms (append tms (list 400)))
    )
  (send *ri* :angle-vector-sequence avs tms)
  
  (ros::unsubscribe "/left_endeffector/wrench")
  (setq *topic-name* "/left_endeffector/wrench")
  (setq *cb-flag* nil)
  (ros::subscribe "/left_endeffector/wrench" geometry_msgs::WrenchStamped #'wrench-x-cb)
  
  (while (some #'identity (send *ri* :wait-interpolation nil 0.1))
    (when (progn
            (ros::spin-once)
            (if *cb-flag* t)
            )
      (send *ri* :cancel-angle-vector)
      (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
      (warn "cancel!!! ~%")
      )
    )
  
  (ros::unsubscribe "/left_endeffector/wrench")

  )

(defun on-stove ()
  (send *ri* :start-grasp :larm :wait t)

  ;; スイッチの位置
  (send *arrow* :newcoords (send *ih-switch-on-coords* :copy-worldcoords))
  (send *arrow* :translate #f(130 0 0) :world)

  (send *pr2* :larm :inverse-kinematics
	(send *arrow* :copy-worldcoords)
	:rotation-axis t
	:debug-view t)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  (push-stove-button)

  (send *arrow* :newcoords (send *ih-switch-on-coords* :copy-worldcoords))
  (send *arrow* :translate #f(130 0 0) :world)
  
  (send *pr2* :larm :inverse-kinematics
	(send *arrow* :copy-worldcoords)
	:rotation-axis t
	:debug-view t)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  
  )

(defun off-stove ()
  (send *ri* :start-grasp :larm :wait t)

  (send *arrow* :newcoords (send *ih-switch-off-coords* :copy-worldcoords))
  (send *arrow* :translate #f(130 0 0) :world)

  (send *pr2* :larm :inverse-kinematics
	(send *arrow* :copy-worldcoords)
	:rotation-axis t
	:debug-view t)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  (push-stove-button)

  (send *arrow* :newcoords (send *ih-switch-off-coords* :copy-worldcoords))
  (send *arrow* :translate #f(130 0 0) :world)
  
  (send *pr2* :larm :inverse-kinematics
	(send *arrow* :copy-worldcoords)
	:rotation-axis t
	:debug-view t)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  )

(defun push-knob ()
  (send *ri* :start-grasp :larm :wait t)

  (send *arrow* :newcoords (send *ih-knob-coords* :copy-worldcoords))
  (send *arrow* :translate #f(130 0 0) :world)

  (send *pr2* :larm :inverse-kinematics
	(send *arrow* :copy-worldcoords)
	:rotation-axis t
	:debug-view t)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  (push-stove-button)
  
  (send *arrow* :newcoords (send *ih-knob-coords* :copy-worldcoords))
  (send *arrow* :translate #f(130 0 0) :world)

  (send *pr2* :larm :inverse-kinematics
	(send *arrow* :copy-worldcoords)
	:rotation-axis t
	:debug-view t)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  
  )

(defun grasp-knob ()
  (send *ri* :stop-grasp :larm :wait t)
  
  (send *arrow* :newcoords (send *ih-knob-coords* :copy-worldcoords))
  (send *arrow* :rotate (deg2rad 90) :x)
  (send *arrow* :translate #f(130 0 0) :world)
  (send *pr2* :larm :inverse-kinematics
	(send *arrow* :copy-worldcoords)
	:rotation-axis t
	:debug-view t)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  
  (push-stove-button)
  
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (send *arrow* :newcoords (send (send *pr2* :larm :end-coords) :copy-worldcoords))
  (send *arrow* :translate #f(10 0 0) :world)
  (send *pr2* :larm :inverse-kinematics
	(send *arrow* :copy-worldcoords)
	:rotation-axis t
	:debug-view t)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  
  (send *ri* :start-grasp :larm :wait t :gain 100)
  )

(defun rotate-knob (&rest args &key (deg 30) &allow-other-keys)
  ;; rotate
  (setq avs (list))
  (setq tms (list))

  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (send *arrow* :newcoords (send (send *pr2* :larm :end-coords) :copy-worldcoords))

  (if (> deg 0)
      (progn 
	(dolist (i (/ deg 10))
	  (send *arrow* :rotate (deg2rad 10) :x)
	  (send *pr2* :larm :inverse-kinematics
		(send *arrow* :copy-worldcoords)
		:rotation-axis t
		:debug-view t)
	  (setq av (send *pr2* :angle-vector))
	  (setq avs (append avs (list av)))
	  (setq tms (append tms (list 600)))
	  ))
    (dolist (i (/ (* deg -1) 10))
      (send *arrow* :rotate (deg2rad -10) :x)
      (send *pr2* :larm :inverse-kinematics
	    (send *arrow* :copy-worldcoords)
	    :rotation-axis t
	    :debug-view t)
      (setq av (send *pr2* :angle-vector))
      (setq avs (append avs (list av)))
      (setq tms (append tms (list 600)))
      )
    )
    
  (send *ri* :angle-vector-sequence avs tms)
  (send *ri* :wait-interpolation)
  
  )

(defun release-knob ()
  (send *ri* :stop-grasp :larm :wait t)
  
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (send *arrow* :newcoords (send (send *pr2* :larm :end-coords) :copy-worldcoords))
  (send *arrow* :translate #f(30 0 0) :world)
  (send *pr2* :larm :inverse-kinematics
	(send *arrow* :copy-worldcoords)
	:rotation-axis t
	:debug-view t)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)

  (send *arrow* :newcoords (send *ih-knob-coords* :copy-worldcoords))
  (send *arrow* :rotate (deg2rad 90) :x)
  (send *arrow* :translate #f(130 0 0) :world)
  (send *pr2* :larm :inverse-kinematics
	(send *arrow* :copy-worldcoords)
	:rotation-axis t
	:debug-view t)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  )


;; pot funcs
(defun grasp-pot-from-above ()
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

  ;; potを上から掴む座標
  (send *arrow* :newcoords (send *pot-larm-from-above* :copy-worldcoords))

  (send *arrow* :translate #f(-100 0 100) :world)
  (send *pr2* :larm :inverse-kinematics
  	(send *arrow* :copy-worldcoords)
  	:rotation-axis t
  	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  (send *ri* :stop-grasp :larm :wait t)

  (send *arrow* :translate #f(100 0 -100) :world)
  (send *pr2* :larm :inverse-kinematics
  	(send *arrow* :copy-worldcoords)
  	:rotation-axis t
  	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  (send *ri* :start-grasp :larm :wait t)
  )

(defun grasp-pot-from-below ()
  ;;腕を引いた姿勢
  (setq *larm0* #f(117.007 48.3805 113.439 -103.326 38.7107 -5.72958 14.4628))
  (send *pr2* :larm :angle-vector *larm0*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)  
  
  ;; potを上から掴む座標
  (send *arrow* :newcoords (send *pot-larm-from-below* :copy-worldcoords))

  (send *arrow* :translate #f(60 0 -60) :world)
  (send *pr2* :larm :inverse-kinematics
  	(send *arrow* :copy-worldcoords)
  	:rotation-axis t
  	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  (send *ri* :stop-grasp :larm :wait t)

  (send *arrow* :translate #f(-60 0 60) :world)
  (send *pr2* :larm :inverse-kinematics
  	(send *arrow* :copy-worldcoords)
  	:rotation-axis t
  	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  (send *ri* :start-grasp :larm :wait t)
  )


;; tool funcs

;; spatula
(defun set-spatula ()
  (setq *spatula* (spatula))

  ;; へらをセットする．
  (send *spatula* :newcoords (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
  (send *spatula* :rotate (deg2rad 90) :x)
  (send *pr2* :rarm :end-coords :assoc *spatula*)

  (objects (list *pr2* *scene* *spatula*))
  (update-view)
  )



(defun mix-with-spatula-set ()
  (send *ri* :start-grasp :rarm :gain 30 :wait t)
  
  (setq *rav0* #f(-20.8048 54.2396 -5.42985 -119.734 197.32 -75.8373 -189.059))
  (send *pr2* :rarm :angle-vector *rav0*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  
  (send *arrow* :newcoords (make-coords :pos (float-vector 600 8550 31145) :rpy (float-vector -3.048 -0.062 -1.634)))
  (send *pr2* :rarm :inverse-kinematics
  	(send *arrow* :copy-worldcoords)
  	:rotation-axis t
  	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  (send *arrow* :newcoords (make-coords :pos (float-vector 550 8400 31150) :rpy (float-vector -1.603 0.03 -0.018)))
  (send *pr2* :rarm :inverse-kinematics
  	(send *arrow* :copy-worldcoords)
  	:move-target (car (send *spatula* :attention))
  	:rotation-axis t
  	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  
  (send *arrow* :newcoords (make-coords :pos (float-vector 471 8170 31070) :rpy (float-vector -1.603 0.03 -0.018))) ;; wait-pose
  (send *pr2* :rarm :inverse-kinematics
  	(send *arrow* :copy-worldcoords)
  	:move-target (car (send *spatula* :attention))
  	:rotation-axis t
  	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  
  (warn "please run (mix-with-spatula) ~%")
  )


(defun mix-with-spatula ()
  ;; 最初の位置
  (send *arrow* :newcoords (make-coords :pos (float-vector 427.739 8071.081 30846.132) :rpy (float-vector -2.267 0.887 0.091)))
  (send *pr2* :rarm :inverse-kinematics
  	(send *arrow* :copy-worldcoords)
  	:move-target (car (send *spatula* :attention))
  	:rotation-axis t
  	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  (send *arrow* :newcoords (make-coords :pos (float-vector 509.423 8110.011 30860.304) :rpy (float-vector -2.038 1.081 0.261)))
  (send *pr2* :rarm :inverse-kinematics
  	(send *arrow* :copy-worldcoords)
  	:move-target (car (send *spatula* :attention))
  	:rotation-axis t
  	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  
  (send *arrow* :newcoords (make-coords :pos (float-vector 486.298 8083.5 30893.89) :rpy (float-vector -1.987 0.871 0.095)))
  (send *pr2* :rarm :inverse-kinematics
  	(send *arrow* :copy-worldcoords)
  	:move-target (car (send *spatula* :attention))
  	:rotation-axis t
  	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (warn "please run (mix-with-spatula) again or (wait-pose-spatula) ~%")
  )

(defun wait-pose-spatula ()
  (send *arrow* :newcoords (make-coords :pos (float-vector 471 8170 31070) :rpy (float-vector -1.603 0.03 -0.018))) ;; wait-pose
  (send *pr2* :rarm :inverse-kinematics
  	(send *arrow* :copy-worldcoords)
  	:move-target (car (send *spatula* :attention))
  	:rotation-axis t
  	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  (warn "please run (mix-with-spatula) or (avoid-rarm-spatula) ~%")
  )

(defun avoid-rarm ()
  (send *arrow* :newcoords (make-coords :pos (float-vector 470 8105 31070) :rpy (float-vector -1.603 0.03 -0.018)))
  (send *pr2* :rarm :inverse-kinematics
  	(send *arrow* :copy-worldcoords)
  	:move-target (car (send *spatula* :attention))
  	:rotation-axis t
  	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  (send *arrow* :newcoords (make-coords :pos (float-vector 550 8450 31150) :rpy (float-vector -1.603 0.03 -0.018)))
  (send *pr2* :rarm :inverse-kinematics
  	(send *arrow* :copy-worldcoords)
  	:move-target (car (send *spatula* :attention))
  	:rotation-axis t
  	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  (send *arrow* :newcoords (make-coords :pos (float-vector 600 8600 31144) :rpy (float-vector -3.048 -0.062 -1.634)))
  (send *pr2* :rarm :inverse-kinematics
  	(send *arrow* :copy-worldcoords)
  	:rotation-axis t
  	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  (setq *rav0* #f(-20.8048 54.2396 -5.42985 -119.734 197.32 -75.8373 -189.059))
  (send *pr2* :rarm :angle-vector *rav0*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  
  (warn "please run (after-transfer) ~%")
  )

;; ladle
(defun set-ladle ()
  (setq *ladle* (ladle))

  ;; おたまをセットする．
  (send *ladle* :newcoords (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
  (send *ladle* :rotate (deg2rad 90) :x)
  (send *pr2* :rarm :end-coords :assoc *ladle*)

  (objects (list *scene* *pr2* *ladle* *pot*))
  (update-view)

  (warn "please run (mix-with-ladle-set) ~%")
  )

(defun mix-with-ladle-set ()
  (send *ri* :start-grasp :rarm :gain 30 :wait t)

  (send *arrow* :newcoords (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
  (send *arrow* :translate #f(0 0 100) :world)
  (send *pr2* :rarm :inverse-kinematics
  	(send *arrow* :copy-worldcoords)
  	:rotation-axis nil
  	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  
  (send *arrow* :newcoords (make-coords :pos (float-vector 550 8250 31180) :rpy (float-vector -1.603 0.0 3.14)))
  (send *pr2* :rarm :inverse-kinematics
  	(send *arrow* :copy-worldcoords)
  	:move-target (car (send *ladle* :attention))
  	:rotation-axis t
  	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  
  (send *arrow* :newcoords (make-coords :pos (float-vector 471 8170 31070) :rpy (float-vector -1.603 0.0 3.14))) ;; wait-pose
  (send *pr2* :rarm :inverse-kinematics
  	(send *arrow* :copy-worldcoords)
  	:move-target (car (send *ladle* :attention))
  	:rotation-axis t
  	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  
  (warn "please run (mix-with-ladle) ~%")
  )

(defun mix-with-ladle ()
  ;; 最初の位置
  (send *arrow* :newcoords (make-coords :pos (float-vector 417.495 8067.248 30861.184) :rpy (float-vector 1.772 1.446 1.068)))
  (send *pr2* :rarm :inverse-kinematics
  	(send *arrow* :copy-worldcoords)
  	:move-target (elt (send *ladle* :attention) 1)
  	:rotation-axis t
  	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  (send *arrow* :newcoords (make-coords :pos (float-vector 540.829 8145.059 30854.487) :rpy (float-vector 0.509 0.96 -0.306)))
  (send *pr2* :rarm :inverse-kinematics
  	(send *arrow* :copy-worldcoords)
  	:move-target (elt (send *ladle* :attention) 1)
  	:rotation-axis t
  	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  
  (send *arrow* :newcoords (make-coords :pos (float-vector 485.52 8113.658 30870.807) :rpy (float-vector 0.77 1.155 -0.115)))
  (send *pr2* :rarm :inverse-kinematics
  	(send *arrow* :copy-worldcoords)
  	:move-target (elt (send *ladle* :attention) 1)
  	:rotation-axis t
  	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (warn "please run (mix-with-ladle) again or (wait-pose-ladle) ~%")
  )

(defun wait-pose-ladle ()
  (send *arrow* :newcoords (make-coords :pos (float-vector 471 8170 31070) :rpy (float-vector -1.603 0.0 3.14))) ;; wait-pose
  (send *pr2* :rarm :inverse-kinematics
  	(send *arrow* :copy-worldcoords)
  	:move-target (car (send *ladle* :attention))
  	:rotation-axis t
  	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  )


(defun now-devel ()
  (grasp-ladle)
  (mix-with-ladle-set)
  (mix-with-ladle)
  )

(now-devel)
