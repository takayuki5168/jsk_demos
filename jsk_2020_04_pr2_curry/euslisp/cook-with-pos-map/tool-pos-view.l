(defun tool-init ()
  ;; init
  (load "move-to-kitchen-with-map.l")

  (load "package://jsk_2020_04_pr2_curry/euslisp/model/knife-new-attention.l")
  (setq *my-knife* (knife))
  (send *my-knife* :newcoords (send *knife-coords* :copy-worldcoords))

  (load "package://jsk_2020_04_pr2_curry/euslisp/model/peeler.l")
  (setq *peeler* (peeler))
  (send *peeler* :newcoords (send *peeler-coords* :copy-worldcoords))

  (load "package://jsk_2020_04_pr2_curry/euslisp/model/potato-peeler.l")
  (setq *potato-peeler* (peeler))
  (send *potato-peeler* :newcoords (send *potato-peeler-coords* :copy-worldcoords))

  (objects (list *pr2* *peeler* *my-knife* *potato-peeler* *scene*))
  (update-view)

  ;; set global num
  (setq *s-time* 3)

  )

(move-to-kitchen-irt)
(tool-init)


;; set codes
(warn "please run (now-set-tools) or (set-knife) ~%")

(defun receive-knife ()
  ;; knifeの位置
  (send *arrow* :newcoords (send *knife-coords* :copy-worldcoords))
  (send *arrow* :translate #f(-12 0 0))
  (send *arrow* :rotate (deg2rad -90) :x)
  
  (send *arrow* :translate #f(60 0 100) :world)
  (send *pr2* :rarm :inverse-kinematics
	(send *arrow* :copy-worldcoords)
	:rotation-axis t
	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  (send *ri* :stop-grasp :rarm)
  (send *ri* :speak-jp "包丁を握らせて下さい")

  (unix:sleep *s-time*)
  (send *ri* :start-grasp :rarm :wait t :gain 100)
  
  (send *knife* :newcoords (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
  (send *knife* :translate #f(12 0 0))
  (send *knife* :rotate (deg2rad 90) :x)
  (send *pr2* :rarm :end-coords :assoc *knife*)
  (send *viewer* :draw-objects)

  (warn "please run (put-knife) ~%")
  )

(defun put-knife ()  
  (send *arrow* :translate #f(-60 0 -100) :world)  
  (send *pr2* :rarm :inverse-kinematics
	(send *arrow* :copy-worldcoords)
	:rotation-axis t
	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  
  (send *ri* :stop-grasp :rarm :wait t)
  (send *pr2* :rarm :end-coords :dissoc *knife*)
  
  ;; 手を上げる
  (send *arrow* :newcoords (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
  (send *arrow* :translate #f(0 0 100) :world)
  (send *pr2* :rarm :inverse-kinematics
	(send *arrow* :copy-worldcoords)
	:rotation-axis t
	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  
  ;; 手を上げる
  (send *arrow* :newcoords (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
  (send *arrow* :translate #f(60 -150 170) :world)
  (send *pr2* :rarm :inverse-kinematics
	(send *arrow* :copy-worldcoords)
	:rotation-axis t
	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  (warn "please run (recieve-peeler) ~%")
  )

(defun receive-peeler ()
  ;;掴む位置
  (send *arrow* :newcoords (send *peeler-coords* :copy-worldcoords))
  (send *arrow* :rotate (deg2rad -90) :x)

  (send *arrow* :translate #f(60 0 100) :world)
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  (send *ri* :stop-grasp :rarm)
  (send *ri* :speak-jp "ピーラーを握らせて下さい")

  (unix:sleep *s-time*)
  (send *ri* :start-grasp :rarm :wait t :gain 100)
  
  (send *peeler* :newcoords (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
  (send *peeler* :rotate (deg2rad 90) :x)
  (send *pr2* :rarm :end-coords :assoc *peeler*)
  (send *viewer* :draw-objects)

  (warn "please run (put-peeler) ~%")
  )

(defun put-peeler ()
  (send *arrow* :translate #f(-60 0 -100) :world)
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  (send *ri* :stop-grasp :rarm :wait t)
  (send *pr2* :rarm :end-coords :dissoc *peeler*)

  ;; 手を上げる
  (send *arrow* :newcoords (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
  (send *arrow* :translate #f(60 0 100) :world)
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  ;; 手を上げる
  (send *arrow* :newcoords (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
  (send *arrow* :translate #f(0 0 80) :world)
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  (warn "please run (recieve-potato-peeler) ~%")
  )

(defun receive-potato-peeler ()
  ;;掴む位置
  (send *arrow* :newcoords (send *potato-peeler-coords* :copy-worldcoords))
  (send *arrow* :rotate (deg2rad -90) :x)

  (send *arrow* :translate #f(60 0 100) :world)
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  (send *ri* :stop-grasp :rarm)
  (send *ri* :speak-jp "ピーラーを握らせて下さい")

  (unix:sleep *s-time*)
  (send *ri* :start-grasp :rarm :wait t :gain 100)
  
  (send *potato-peeler* :newcoords (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
  (send *potato-peeler* :rotate (deg2rad 90) :x)
  (send *pr2* :rarm :end-coords :assoc *potato-peeler*)
  (send *viewer* :draw-objects)

  (warn "please run (put-potato-peeler) ~%")
  )


(defun put-potato-peeler ()
  (send *arrow* :translate #f(-60 0 -100) :world)
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  (send *ri* :stop-grasp :rarm :wait t)
  (send *pr2* :rarm :end-coords :dissoc *potato-peeler*)

  ;; 手を上げる
  (send *arrow* :newcoords (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
  (send *arrow* :translate #f(60 0 100) :world)
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  ;; 手を上げる
  (send *arrow* :newcoords (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
  (send *arrow* :translate #f(0 0 80) :world)
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  (warn "please run (reset-rarm) ~%")
  )


(defun reset-rarm ()

  (setq *rav0* #f(-20.8048 54.2396 -5.42985 -119.734 197.32 -75.8373 -189.059))
  (send *pr2* :rarm :angle-vector *rav0*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (setq *rav2* #f(-30.0303 1.63624 -169.166 -116.783 100.843 -98.3015 -58.1037))
  (send *pr2* :rarm :angle-vector *rav2*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  )

(defun up-larm ()
  ;; 手を上げる
  (send *arrow* :newcoords (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
  (send *arrow* :translate #f(0 0 50) :world)
  (send *pr2* :rarm :inverse-kinematics
        (send *arrow* :copy-worldcoords)
        :rotation-axis t
        :debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  )


(defun now-set-tools ()
  (up-larm)
  (receive-knife)
  (put-knife)
  (receive-peeler)
  (put-peeler)
  (receive-potato-peeler)
  (put-potato-peeler)  
  )

(defun set-knife ()
  (up-larm)
  (receive-knife)
  (put-knife)
  )

(defun set-peeler ()
  (up-larm)
  (receive-peeler)
  (put-peeler)  
  )

(defun set-potato-peeler ()
  (up-larm)
  (receive-potato-peeler)
  (put-potato-peeler)  
  )
