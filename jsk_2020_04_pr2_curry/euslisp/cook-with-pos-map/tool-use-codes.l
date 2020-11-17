(defun tool-init ()
  ;; init
  (load "move-to-kitchen-with-map.l")

  (load "package://jsk_2020_04_pr2_curry/euslisp/model/knife-new-attention.l")
  (setq *knife* (knife))
  (send *knife* :newcoords (send *knife-coords* :copy-worldcoords))

  (load "package://jsk_2020_04_pr2_curry/euslisp/model/peeler.l")
  (setq *peeler* (peeler))
  (send *peeler* :newcoords (send *peeler-coords* :copy-worldcoords))

  (load "package://jsk_2020_04_pr2_curry/euslisp/model/potato-peeler.l")
  (setq *potato-peeler* (peeler))
  (send *potato-peeler* :newcoords (send *potato-peeler-coords* :copy-worldcoords))

  (load "package://jsk_2020_04_pr2_curry/euslisp/model/spatula.l")
  (setq *spatula* (spatula))
  (send *spatula* :newcoords (send *spatula-coords-1* :copy-worldcoords)) ;; tmp 本来は棚の中想定

  (load "package://jsk_2020_04_pr2_curry/euslisp/model/ladle.l")
  (setq *ladle* (ladle))
  (send *ladle* :newcoords (send *ladle-coords-1* :copy-worldcoords)) ;; tmp 本来は棚の中想定

  (move-to-kitchen-irt)
  (objects (list *pr2* *peeler* *knife* *potato-peeler* *scene* *spatula* *ladle*))
  (update-view)
  
  ;; set global num
  (setq *s-time* 3)
  
  )

(tool-init)


;; set codes
(warn "If you use real robot, run (update-pos) ~%")
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


(defun receive-spatula ()
  ;; spatulaの位置
  (send *arrow* :newcoords (send *spatula-coords-1* :copy-worldcoords))
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
  
  (send *spatula* :newcoords (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
  (send *spatula* :rotate (deg2rad 90) :x)
  (send *pr2* :rarm :end-coords :assoc *spatula*)
  (send *viewer* :draw-objects)

  (warn "please run (put-spatula-1) ~%")
  )

(defun put-spatula-1 ()  
  (send *arrow* :newcoords (send *spatula-coords-1* :copy-worldcoords))
  (send *arrow* :rotate (deg2rad -90) :x)
  
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
  
  (send *ri* :stop-grasp :rarm :wait t)
  (send *pr2* :rarm :end-coords :dissoc *spatula*)
  
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
  )

(defun receive-ladle ()
  ;; ladleの位置
  (send *arrow* :newcoords (send *ladle-coords-1* :copy-worldcoords))
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
  
  (send *ladle* :newcoords (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
  (send *ladle* :rotate (deg2rad 90) :x)
  (send *pr2* :rarm :end-coords :assoc *ladle*)
  (send *viewer* :draw-objects)

  (warn "please run (put-ladle-1) ~%")
  )

(defun put-ladle-1 ()  
  (send *arrow* :newcoords (send *ladle-coords-1* :copy-worldcoords))
  (send *arrow* :rotate (deg2rad -90) :x)
  
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
  
  (send *ri* :stop-grasp :rarm :wait t)
  (send *pr2* :rarm :end-coords :dissoc *ladle*)
  
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
  )

(defun open-shelf-set ()
  ;; 中間姿勢になるための座標
  (send *arrow* :newcoords (make-coords :pos (float-vector 570 8500 31100) :rpy (float-vector 3.14 0.0 0.0)))
  (send *pr2* :rarm :inverse-kinematics
	(send *arrow* :copy-worldcoords)
	:rotation-axis t
	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  ;;グリッパを微妙に開く
  (send *ri* :move-gripper :rarm 0.05)
  
  ;; 棚のノブの位置
  (send *arrow* :newcoords (send *shelf-knob-closed-coords* :copy-worldcoords))

  (send *arrow* :translate (float-vector 100 0 -100) :world)
  (send *pr2* :rarm :inverse-kinematics
	(send *arrow* :copy-worldcoords)
	:rotation-axis t
	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  (send *arrow* :translate (float-vector 0 0 100) :world)
  (send *pr2* :rarm :inverse-kinematics
	(send *arrow* :copy-worldcoords)
	:rotation-axis t
	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  ;; ノブのところまで手を伸ばす
  (setq avs (list))
  (setq tms (list))
  
  (dotimes (i 10)
    (send *arrow* :translate (float-vector -10 0 0) :world)
    (send *pr2* :rarm :inverse-kinematics
          *arrow*
          :rotation-axis t
          :debug-view t)
    
    (setq av (send *pr2* :angle-vector))
    
    (setq avs (append avs (list av)))
    (setq tms (append tms (list 600)))
    )
  
  (send *ri* :angle-vector-sequence avs tms)
  (send *ri* :wait-interpolation)
  
  (send *ri* :start-grasp :rarm :wait t)
  
  (warn "please run (open-shelf) ~%")
  )

(defun open-shelf ()
  ;; 電子レンジの扉開けをパクって実装する
  
  (let ((i 170)   ;; open
	(av)
	(tms (list))
	(avs (list)))
    (while (> i 89) ;; 99
      (send *microwave* :angle-vector (float-vector i))
      (setq handle-coords (make-coords :pos (send (send m-handle :copy-worldcoords) :worldpos) :rpy (float-vector (car (car (rpy-angle (send (send m-handle :copy-worldcoords) :worldrot)))) 0 0)))
      (send handle-coords :translate (float-vector -80 -10 50))
      
      (send *pr2* :larm :inverse-kinematics (send (send handle-coords :copy-worldcoords) :rotate (/ pi -2) :y) :rotation-axis t)
      (setq av (send *pr2* :angle-vector))
      (setq avs (append avs (list av)))
      (setq tms (append tms (list 600)))
      
      (setq i (- i 10))
      )
    (send *ri* :angle-vector-sequence avs tms)
    (send *ri* :wait-interpolation)
    )
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
  
  (setq *rav3* #f(-75.0 50.0 -110.0 -110.0 20.0 -10.0 -10.0))
  (send *pr2* :rarm :angle-vector *rav3*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  )

(defun up-rarm ()
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

;; grasp-codes
(defun grasp-tool-on-table ()
  ;; arrow　はセットしてある前提
  (send *ri* :stop-grasp :rarm :wait t)

  (send *arrow* :translate #f(50 0 100) :world)
  (send *pr2* :rarm :inverse-kinematics
	(send *arrow* :copy-worldcoords)
	:rotation-axis t
	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  
  (send *arrow* :translate #f(-50 0 -100) :world)  
  (send *pr2* :rarm :inverse-kinematics
	(send *arrow* :copy-worldcoords)
	:rotation-axis t
	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)  

  (send *ri* :start-grasp :rarm :wait t)
  )


(defun grasp-knife ()
  (up-rarm)

  ;;掴む位置
  (send *arrow* :newcoords (send *knife* :copy-worldcoords))
  (send *arrow* :translate #f(-12 0 0))
  (send *arrow* :rotate (deg2rad -90) :x)

  ;; つかむ
  (grasp-tool-on-table)  
  (send (send *pr2* :rarm :end-coords) :assoc *knife*)
  
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

  ;; 手を上げる
  (send *arrow* :newcoords (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
  (send *arrow* :translate #f(40 0 150) :world)
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
  )

(defun grasp-peeler ()
  (up-rarm)
  
  ;;掴む位置
  (send *arrow* :newcoords (send *peeler* :copy-worldcoords))
  (send *arrow* :rotate (deg2rad -90) :x)
  
  (send *ri* :stop-grasp :rarm :wait t)
  
  ;; つかむ
  (grasp-tool-on-table)  
  (send (send *pr2* :rarm :end-coords) :assoc *peeler*)
  
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

  ;; 手を上げる
  (send *arrow* :newcoords (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
  (send *arrow* :translate #f(40 0 150) :world)
  (send *pr2* :rarm :inverse-kinematics
	(send *arrow* :copy-worldcoords)
	:rotation-axis t
	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  )


(defun grasp-potato-peeler ()
  (up-rarm)

  ;;掴む位置
  (send *arrow* :newcoords (send *potato-peeler* :copy-worldcoords))
  (send *arrow* :rotate (deg2rad -90) :x)

  ;; つかむ
  (grasp-tool-on-table)  
  (send (send *pr2* :rarm :end-coords) :assoc *potato-peeler*)
  
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

  ;; 手を上げる
  (send *arrow* :newcoords (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
  (send *arrow* :translate #f(40 0 150) :world)
  (send *pr2* :rarm :inverse-kinematics
	(send *arrow* :copy-worldcoords)
	:rotation-axis t
	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)  
  )


(defun grasp-spatula ()
  (up-rarm)
  
  ;;掴む位置
  (send *arrow* :newcoords (send *spatula* :copy-worldcoords))
  (send *arrow* :rotate (deg2rad -90) :x)
  
  (send *ri* :stop-grasp :rarm :wait t)
  
  ;; つかむ
  (grasp-tool-on-table)  
  (send (send *pr2* :rarm :end-coords) :assoc *spatula*)
  
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

  ;; 手を上げる
  (send *arrow* :newcoords (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
  (send *arrow* :translate #f(40 0 150) :world)
  (send *pr2* :rarm :inverse-kinematics
	(send *arrow* :copy-worldcoords)
	:rotation-axis t
	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  )

(defun grasp-ladle ()
  (up-rarm)
  
  ;;掴む位置
  (send *arrow* :newcoords (send *ladle* :copy-worldcoords))
  (send *arrow* :rotate (deg2rad -90) :x)
  
  (send *ri* :stop-grasp :rarm :wait t)
  
  ;; つかむ
  (grasp-tool-on-table)  
  (send (send *pr2* :rarm :end-coords) :assoc *ladle*)
  
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

  ;; 手を上げる
  (send *arrow* :newcoords (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
  (send *arrow* :translate #f(40 0 150) :world)
  (send *pr2* :rarm :inverse-kinematics
	(send *arrow* :copy-worldcoords)
	:rotation-axis t
	:debug-view t)
  (send *viewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  )








(defun now-set-tools ()
  (up-rarm)
  (receive-knife)
  (put-knife)
  (receive-peeler)
  (put-peeler)
  (receive-potato-peeler)
  (put-potato-peeler)  
  )

(defun set-knife ()
  (up-rarm)
  (receive-knife)
  (put-knife)
  )

(defun set-peeler ()
  (up-rarm)
  (receive-peeler)
  (put-peeler)  
  )

(defun set-potato-peeler ()
  (up-rarm)
  (receive-potato-peeler)
  (put-potato-peeler)  
  )

(defun set-spatula-1 ()
  (up-rarm)
  (receive-spatula-1)
  (put-spatula-1)
  )

(defun set-ladle-1 ()
  (up-rarm)
  (receive-ladle-1)
  (put-ladle-1)
  )
