;; (load "ladle.l")
;; (setq *otama* (ladle))

;; (load "models/arrow-object.l")
;; (setq *arrow* (arrow))

;; (send *arrow* :newcoords (send (elt (send *otama* :attention) 1) :copy-worldcoords))
;; (objects (list *otama* *arrow*))



(load "spatula.l")
(setq *otama* (spatula))

(load "models/arrow-object.l")
(setq *arrow* (arrow))

(send *arrow* :newcoords (send (elt (send *otama* :attention) 1) :copy-worldcoords))
(objects (list *otama* *arrow*))
