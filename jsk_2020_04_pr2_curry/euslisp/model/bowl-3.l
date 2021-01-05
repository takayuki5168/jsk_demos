(defclass bowl-3-object
  :super cascaded-link
  :slots (
         ))

(defmethod bowl-3-object
  (:init
   (&rest args &key (name "bowl-3") (pos (float-vector 0 0 0)) (rot (unit-matrix 3)) &allow-other-keys)
   (let (link1 b1 b2 b3 b4 b5)
     (send-super* :init :name name args)
     
     ;; definition of link
     (setq b1 (make-sphere 78))
     (setq b2 (make-cube 1000 1000 100))
     (send b2 :locate #f(0 0 38))
     (setq b3 (make-sphere 76))
     (setq b4 (make-cylinder 47 5))
     (setq b5 (make-cube 1000 1000 40))
     (send b5 :locate #f(0 0 -80))
     
     (setq b1 (body- b1 b2))
     (setq b1 (body- b1 b3))
     (setq b1 (body- b1 b5))
     
     (send b1 :locate #f(0 0 60))
     (send b4 :locate #f(0 0 0))
     
     (send b1 :set-color :Green)
     (send b4 :set-color :Green) 
     
     (setq link1 (instance bodyset-link :init (make-cascoords) :bodies (list b1 b4) :weight 1 :centroid (float-vector 0.0 0.0 0.0) :inertia-tensor #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0))))


     ;; definition of assoc
     (send self :assoc link1)

     ;; definition of end-coords

     ;; definition of joint

     ;; definition of :handle

     ;; init-ending 
     (setq links (list link1))
     (send self :init-ending)
     (send self :move-to (make-coords :pos pos :rot rot))
     (send-all links :worldcoords)

     self))

  )

(defun bowl-3 (&rest args) (instance* bowl-3-object :init args))