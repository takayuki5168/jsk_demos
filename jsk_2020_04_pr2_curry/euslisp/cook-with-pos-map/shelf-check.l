(require :eng2-scene "package://jsk_2020_04_pr2_curry/euslisp/model/my-eng2-scene.l")

(load "package://jsk_2020_04_pr2_curry/euslisp/model/shelf-door.l")
(setq *shelf-door* (shelf-door))
(send *shelf-door* :newcoords (send *shelf-door-coords* :copy-worldcoords))


(objects (list *shelf-door*))

(warn "please run (open) or (close) ~%")

(defun open ()
  (send *shelf-door* :angle-vector #f(90))
  (send *viewer* :draw-objects)
  (print (send *shelf-door* :handle))

  (warn "please run (close) ~%")
  )

(defun close ()
  (send *shelf-door* :angle-vector #f(0))
  (send *viewer* :draw-objects)
  (print (send *shelf-door* :handle))

  (warn "please run (open) ~%")
  )
