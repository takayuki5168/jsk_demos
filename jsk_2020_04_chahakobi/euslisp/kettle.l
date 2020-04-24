(load "models/kettle-object.l")
(setq *kettle* (kettle))

(load "models/arrow-object.l")
(setq *axis* (arrow))

;;(send *axis* :newcoords (send *kettle* :worldcoords))


(objects (list *kettle* *axis*))
