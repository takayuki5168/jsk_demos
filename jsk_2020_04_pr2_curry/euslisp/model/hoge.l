(setq b1 (make-cube 12 100 12))

(setq b2 (make-cylinder 30 5))
(setq b3 (make-cylinder 25 5))
(setq b2 (body- b2 b3))
(setq b4 (make-cube 100 100 20))
(send b4 :locate #f(0 51 0))
(setq b2 (body- b2 b4))

(setq b5 (make-cube 50 5 2))
(send b5 :locate #f(0 0 2))
(send b5 :set-color :white)

(send b2 :assoc b5)
(send b2 :locate #f(0 80 0))

(send b1 :assoc b2)
(send b2 :set-color :black)
(send b1 :set-color :black)

(objects (list b1 b2 b5))

