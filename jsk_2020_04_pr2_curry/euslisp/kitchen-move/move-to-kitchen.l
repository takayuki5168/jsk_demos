(load "../model/my-room73b2-scene.l")
(setq *room* (room73b2))

(load "package://pr2eus/pr2-interface.l")
(pr2-init)


;; set spots
(setq *cook-spot* (send *room73b2* :spot "cook-spot"))
(setq *sink-spot* (make-coords :pos #f(1055 2600 0) :rpy #f(3.14 0 0))) 
(setq *desk-spot* (make-coords :pos #f(3470 -1900 0) :rpy #f(-1.57 0 0)))

(setq *kitchen-spot* (send *room73b2* :spot "kitchen-spot"))


;; キッチンの場所まで移動

(defun move-to-kitchen-irt ()
  (send *pr2* :move-to *kitchen-spot* :world)
  )

(defun move-to-kitchen-ri ()
    (send *ri* :speak-jp "台所まで移動します")
    (send *ri* :clear-costmap)
    (send *ri* :move-to (make-coords :pos #f(1100 8500 351) :rpy (float-vector pi 0 0)) :frame-id "/map") ;; need change value 
    (send *ri* :wait-interpolation))


(move-to-kitchen-irt)

(objects (list *room* *pr2*))
(send *irtviewer* :look-all
      (geo::make-bounding-box 
       (flatten (send-all (send *pr2* :bodies) :vertices))))


(warn "please run (move-to-kitchen-ri)~%")
;; (move-to-kitchen-ri)
