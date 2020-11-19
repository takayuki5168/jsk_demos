(defclass shelf-door-object
  :super cascaded-link
  :slots (
	  handle0
         ))

(defmethod shelf-door-object
  (:init
   (&rest args &key (name "shelf-door") (pos (float-vector 0 0 0)) (rot (unit-matrix 3)) &allow-other-keys)
   (let (link1 b1 b2)
     (send-super* :init :name name args)

     ;; definition of link
     (setq b1 (make-cube 10 295 60))     
     (setq b2 (make-cube 10 10 30))

     (send b1 :locate #f(-5 147.5 0))
     (send b2 :locate #f(-15 275 0))

     (send b1 :set-color :blue)
     (send b2 :set-color :yellow)
     (send b1 :assoc b2)
     

     (setq link1 (instance bodyset-link :init (make-cascoords) :bodies (list b1 b2) :weight 1 :centroid (float-vector 0.0 0.0 0.0) :inertia-tensor #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0))))

     ;; link0
     (setq b3 (make-cube 100 10 60))
     (send b3 :locate #f(50 0 0))
     (send b3 :set-color :blue)
     
     (setq link0 (instance bodyset-link :init (make-cascoords) :bodies (list b3) :weight 1 :centroid (float-vector 0.0 0.0 0.0) :inertia-tensor #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0))))
     
     ;; definition of assoc
     (send self :assoc link1)

     ;; definition of end-coords

     ;; definition of joint

     ;; definition of :handle
     (setq handle0 (make-cascoords :pos (float-vector 0.0 0.0 0.0) :rot #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0)) :name "handle0"))
     (send link1 :assoc handle0)

     ;; definition of :attention
     (setq attention0 (make-cascoords :pos (float-vector 260 0 0) :rot #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0)) :name "attention0"))
     (send link1 :assoc attention0)
     (setq attention1 (make-cascoords :pos (float-vector 260 0 0) :rot #2f((0.965926 -0.258819 0.0) (0.258819 0.965926 0.0) (0.0 0.0 1.0)) :name "attention1"))
     (send link1 :assoc attention1)


     ;; init-ending 
     (setq links (list link1 link0))
     (send self :init-ending)
     (send self :move-to (make-coords :pos pos :rot rot))
     (send-all links :worldcoords)

     self))

  (:handle (&rest args) (forward-message-to-all (list  handle0) args))
  (:handle-handle0 (&rest args) (forward-message-to handle0 args))
  (:attention (&rest args) (forward-message-to-all (list attention0 attention1) args))
  (:attention-attention0 (&rest args) (forward-message-to-all attention0 args))
  (:attention-attention1 (&rest args) (forward-message-to-all attention1 args))
  )

(defun shelf-door (&rest args) (instance* shelf-door-object :init args))



;; (defclass microwave-model
;;     :super cascaded-link
;;   :slots (m-body m-door m-joint m-handle))

;; (defmethod microwave-model
;;     (:init
;;      ()
;;      (let ((lx 320)
;;            (ly 424)
;;            (lz 255)
;;            (wd 34)
;;            (w 30))

;;        (send-super :init)

;;        (setq m-body (instance bodyset-link :init (make-cascoords) :bodies (list (make-body lx ly lz w)) :locate (float-vector 0 (/ ly 2) 0)))
;;        ;;(send m-body :locate (float-vector 0 (/ ly 2) 0))

;;        (setq m-door (instance bodyset-link :init (make-cascoords) :bodies (list (make-door ly lz wd)) :locate #f(100 0 0)))
;;        ;;(send m-door :locate (float-vector 0 0 0))

;;        (setq m-handle (instance bodyset-link :init (make-cascoords) :bodies (list (make-handle ly)) :locate (float-vector 0  ly 0)))
;;        ;;(send m-handle :locate (float-vector 0 (/ ly 2) 0))

;;        (setq m-joint (instance rotational-joint :init :parent-link m-body :child-link m-door :axis :z :min -180 :max 360))

;;        (setq links (list m-body m-door))
;;        (setq joint-list (list m-joint))

;;        (send self :assoc m-body)
;;        (send m-body :assoc m-door)
;;        (send m-door :assoc m-handle)

;;        (send self :init-ending)
;;        self))

;;   (:get-handle () m-handle)

;;   (:m-joint (&rest args) (forward-message-to m-joint args))
;;   )

;; (defun microwave-init
;;     ()
;;   (setq inst (instance microwave-model :init))
;;   (send inst :angle-vector (float-vector 180))
;;   inst)
