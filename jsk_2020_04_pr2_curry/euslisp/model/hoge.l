;; ;; ladle
;; (load "ladle.l")
;; (setq *otama* (ladle))

;; (load "models/arrow-object.l")
;; (setq *arrow* (arrow))

;; (send *arrow* :newcoords (send (elt (send *otama* :attention) 1) :copy-worldcoords))
;; (objects (list *otama* *arrow*))


;; ;; spatula
;; (load "spatula.l")
;; (setq *otama* (spatula))

;; (load "models/arrow-object.l")
;; (setq *arrow* (arrow))

;; (send *arrow* :newcoords (send (elt (send *otama* :attention) 1) :copy-worldcoords))
;; (objects (list *otama* *arrow*))



(load "ladle.l")
(setq *otama* (ladle))

(load "models/arrow-object.l")
(setq *arrow* (arrow))

(send *arrow* :newcoords (send (elt (send *otama* :attention) 1) :copy-worldcoords))

(load "spatula.l")
(setq *spatula* (spatula))

(objects (list *otama* *arrow* *spatula*))
