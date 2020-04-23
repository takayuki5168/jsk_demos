(load "measuring-cup.l")
(setq *hoge* (measuring-cup))

(load "models/arrow-object.l")
(setq *axis* (arrow))

;;(send *axis* :newcoords (car (send *hoge* :handle))) 
;;(send *axis* :newcoords (make-coords :pos #f(0 -50 110)))
(send *axis* :newcoords (car (send *hoge* :attention))) 

(objects (list *hoge* *axis*))
