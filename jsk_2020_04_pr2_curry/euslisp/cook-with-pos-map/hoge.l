(load "tool-use-codes.l")
;; (send *arrow* :newcoords (send *shelf-knob-closed-coords* :copy-worldcoords))
(send *arrow* :newcoords (send (car (send *shelf-door* :handle)) :copy-worldcoords))
(objects *arrow*)
(update-view)

(warn "please run (open-test) ~%")

(defun open-test ()
  (send *shelf-door* :angle-vector #f(90))
  (send *arrow* :newcoords (send (car (send *shelf-door* :handle)) :copy-worldcoords))
  (update-view)
  )
