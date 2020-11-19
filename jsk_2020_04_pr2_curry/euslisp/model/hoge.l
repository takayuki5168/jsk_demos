(load "shelf-door.l")
(setq *shelf-door* (shelf-door))
(send *shelf-door* :angle-vector #f(10))

(load "models/arrow-object.l")
(setq *arrow* (arrow))
(send *arrow* :newcoords (send (car (send *shelf-door* :handle)) :copy-worldcoords))

(objects (list *shelf-door* *arrow*))
