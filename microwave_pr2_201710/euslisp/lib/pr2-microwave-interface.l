(require :pr2-interface "package://pr2eus/pr2-interface.l")
(require :microwave "package://microwave_pr2_201710/euslisp/lib/microwave.l")

(require :eng2-scene "package://jsk_maps/src/eng2-scene.l")
(require :util "package://microwave_pr2_201710/euslisp/lib/util.l")

(require :detection-interface "package://jsk_perception/euslisp/detection_interface.l")
(defparameter *detection-topic* "/ObjectDetection")

(defclass pr2-microwave-interface
  :super object
  :slots (microwave-pos microwave-rpy)
  )

(defmethod pr2-microwave-interface
<<<<<<< HEAD
    (:init
     (&optional (use-ri- nil))

        (setq use-ri use-ri-)
        ;; init ROS Node
        (ros::roseus "pr2-microwave")

        ;; init detection
        ;;(defparameter *detection-topic* "/ObjectDetection")

        ;; init 73B2 room
        (unless (boundp '*scene*) (setq *scene* (make-eng2-scene)))

        ;; init PR2
        (unless (boundp '*pr2*) (setq *pr2* (instance pr2-sensor-robot :init)))
        (if use-ri
            (unless (boundp '*ri*) (setq *ri* (instance pr2-interface :init))))
        (if use-ri
            (send *pr2* :move-to (send *ri* :state :worldcoords) :world))

	;;KO
	;;(setq av-ri (send *ri* :state :potentio-vector))
	;;(send *pr2* :angle-vector av-ri)
	(send *pr2* :angle-vector #f(50.0193 59.9994 74.0014 70.0087 -119.999 20.0076 -30.0019 180.001 -39.4936 26.0339 -93.5104 -120.297 -203.777 -58.4382 258.91 7.7887 28.32))
	(send *pr2* :torso-waist-z :joint-angle 50)
	(send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
        (send *ri* :wait-interpolation)
	

        ;; init microwave
        (unless (boundp '*microwave*) (setq *microwave* (microwave-init)))
        (send *microwave* :move-to (send (send *scene* :spot "/eng2/7f/room73B2-counter-side") :worldcoords) :world)
        (send *microwave* :move-to (make-coords :pos #f(0 0 1000)))
        ;;(send *microwave* :move-to (make-coords :pos #f(2000 7457 31000) :rpy (float-vector (/ pi 2.0) 0 0)) :world)

        ;; show objects in irtviewer
        (objects (list *scene* *microwave* *pr2*))
        (send *irtviewer* :look-all *pr2*)

        ;; register spot of microwave
        (let ((microwave-front-spots (make-cascoords :pos #f(2280 7100 30000) :rpy (float-vector (/ pi 2.0) 0 0) :name "microwave-front-spot")))
          (send *scene* :add-spots (list microwave-front-spots))
          )
        )
=======
  (:init
   ()

   ;; init ROS Node
   (ros::roseus "pr2-microwave")

   ;; init 73B2 room
   (unless (boundp '*scene*) (setq *scene* (make-eng2-scene)))

   ;; init PR2
   (unless (boundp '*pr2*) (setq *pr2* (instance pr2-sensor-robot :init)))
   (unless (boundp '*ri*) (setq *ri* (instance pr2-interface :init)))
   (send *pr2* :move-to (send *ri* :state :worldcoords) :world)

   ;; init microwave
   (unless (boundp '*microwave*) (setq *microwave* (microwave-init)))
   (send *microwave* :move-to (send (send *scene* :spot "/eng2/7f/room73B2-counter-side") :worldcoords) :world)
   (send *microwave* :move-to (make-coords :pos (float-vector 0 0 1000)))

   ;; show objects in irtviewer
   (objects (list *scene* *microwave* *pr2*))
   (send *irtviewer* :look-all *pr2*)

   ;; register spot of microwave
   (let ((microwave-front-spots (make-cascoords :pos (float-vector 2280 7100 30000) :rpy (float-vector (/ pi 2.0) 0 0) :name "microwave-front-spot")))
     (send *scene* :add-spots (list microwave-front-spots))
     )
   )
>>>>>>> 1d91b1e3... formatting by emacs

  (:go-forward
   ()
   (send *ri* :go-pos-unsafe 0.15 0 0)
   )

  (:go-backward
   ()
   (send *ri* :go-pos-unsafe -0.15 0 0)
   )

  (:go-to-microwave-roughly
   ()
   (send *ri* :move-to (send *scene* :spot "microwave-front-spot") :frame-id "world")

   (send *microwave* :move-to (send (send *scene* :spot "/eng2/7f/room73B2-counter-side") :worldcoords) :world)
   (send *microwave* :move-to (make-coords :pos (float-vector 0 0 1000)))

   (send *pr2* :move-to (send (send *scene* :spot "/eng2/7f/room73B2-counter-side") :worldcoords) :world)
   (send *pr2* :move-to (make-coords :pos (float-vector -700 0 0)))
   (pr2-left-reset-pose)

   (send *irtviewer* :look-all *pr2*)
   (send *irtviewer* :look-all *pr2*)
   )


  (:go-to-microwave-accurately
   ()
   ;;KO instead of moving PR2 in init the torso is moved down here
   ;;(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
   ;;(send *ri* :wait-interpolation)
   ;;KO make sure that cup does not crash cupboard
   (send *pr2* :rarm :angle-vector #f(-39.4936 26.0339 -93.5104 -120.297 -203.777 -6.0 258.91))
   (when use-ri
       (send *ri* :angle-vector (send *pr2* :angle-vector) 1500)
       (send *ri* :wait-interpolation))
   
   (let ((recog-flag nil)
         (cds))
<<<<<<< HEAD
     (if (null use-ri)
         (progn
           (setq microwave-pos (send *microwave* :pos))
           (setq microwave-rpy (send *microwave* :rpy-angle))

           (send *microwave* :move-to (make-coords :pos microwave-pos :rpy (car microwave-rpy)) :world)
           (send *microwave* :translate (float-vector -185 450 -240))  ;; x奥行き y横 z高さ
           )
       (progn
         (while (null recog-flag)
           (send *microwave* :move-to (send *ri* :state :worldcoords) :world)
           (send *pr2* :move-to (send *ri* :state :worldcoords) :world)

           (send *pr2* :head :neck-p :joint-angle 28)
           (send *ri* :angle-vector (send *pr2* :angle-vector))
           (send *ri* :wait-interpolation)

           (setq cds (check-detection :type "microwave" :speak-name "denshirenji" :timeout 10 :speak nil))
           (send *microwave* :move-to (make-coords :pos (send cds :pos)))
           (setq microwave-pos (send *microwave* :pos))
           (setq microwave-rpy (send *microwave* :rpy-angle))

           (let* ((local-microwave-x (elt (send cds :worldpos) 0))
                  (local-microwave-y (elt (send cds :worldpos) 1))
                  (local-microwave-theta (car (car (rpy-angle (send cds :worldrot)))))
                  (local-microwave-without-z (make-coords :pos (float-vector local-microwave-x local-microwave-y 0) :rpy (list local-microwave-theta 0 0)))
                  (world-microwave-without-z ;; pose of microwave in world when z = 0
                   (send (send (send *pr2* :worldcoords) :copy-worldcoords) :transform local-microwave-without-z))
                  (relative-coords
                   (make-coords :pos (float-vector 600 0 0)))
                  (reach-world-microwave-without-z ;; pose in front of microwave in world when z = 0
                   (send (send world-microwave-without-z :copy-worldcoords) :transform relative-coords))
                  (reach-local-microwave-local-without-z
                   (send (send (send *pr2* :worldcoords) :copy-worldcoords)
                         :transformation reach-world-microwave-without-z))
                  )
             (send world-microwave-without-z :draw-on :flush t :size 200 :width 5 :color #f(1 0 0))
             (send reach-world-microwave-without-z :draw-on :flush t :size 200 :width 5 :color #f(0 1 0))
             (let* ((go-pos-x (* 1e-3 (elt (send reach-local-microwave-local-without-z :worldpos) 0)))
                    (go-pos-y (* 1e-3 (elt (send reach-local-microwave-local-without-z :worldpos) 1)))
                    (go-pos-theta (+ (rad2deg (car (car (rpy-angle (send reach-local-microwave-local-without-z :worldrot))))) 180))
                    )
<<<<<<< HEAD
               (send world-microwave-without-z :draw-on :flush t :size 200 :width 5 :color #f(1 0 0))
               (send reach-world-microwave-without-z :draw-on :flush t :size 200 :width 5 :color #f(0 1 0))
               (let* ((go-pos-x (* 1e-3 (elt (send reach-local-microwave-local-without-z :worldpos) 0)))
                      (go-pos-y (* 1e-3 (elt (send reach-local-microwave-local-without-z :worldpos) 1)))
                      (go-pos-theta (+ (rad2deg (car (car (rpy-angle (send reach-local-microwave-local-without-z :worldrot))))) 180))
                      )
                 (if (> go-pos-theta 180)
                     (setq go-pos-theta (- go-pos-theta 360)))
                 (if (< go-pos-theta -180)
                     (setq go-pos-theta + go-pos-theta 360))

                 (format t "go-pos x:~a y:~a theta:~a~%"
                         go-pos-x go-pos-y go-pos-theta)
                 ;;(if (and (< (abs go-pos-x) 0.05) (< (abs go-pos-y) 0.05) (< (abs go-pos-theta) 0.8))
                 (if (and (< (abs go-pos-x) 0.05) (< (abs go-pos-y) 0.05) (< (abs go-pos-theta) 1.2))
                     (setq recog-flag t)
                     ;;(when (y-or-n-p "send to real robot?")
                     (progn
                       (let (go-pos-x- go-pos-y- go-pos-theta-)
                         ;; clipping how robot moves because using :go-pos-`unsafe`
<<<<<<< HEAD
                         (setq go-pos-x- (min go-pos-x 0.05))
                         (setq go-pos-x- (max go-pos-x- -0.05))
                         (setq go-pos-y- (min go-pos-y 0.05))
                         (setq go-pos-y- (max go-pos-y- -0.05))
=======
                         (setq go-pos-x- (min go-pos-x 0.1))
                         (setq go-pos-x- (max go-pos-x- -0.1))
                         (setq go-pos-y- (min go-pos-y 0.1))
                         (setq go-pos-y- (max go-pos-y- -0.1))
>>>>>>> 3f3ed6c5... some implementation
                         (setq go-pos-theta- (min go-pos-theta 3))
                         (setq go-pos-theta- (max go-pos-theta- -3))

                         (send *ri* :go-pos-unsafe go-pos-x- go-pos-y- go-pos-theta-)
                         )
                       )
                     ;;)
=======
               (if (> go-pos-theta 180)
                   (setq go-pos-theta (- go-pos-theta 360)))
               (if (< go-pos-theta -180)
                   (setq go-pos-theta + go-pos-theta 360))

               (format t "go-pos x:~a y:~a theta:~a~%"
                       go-pos-x go-pos-y go-pos-theta)
               (if (and (< (abs go-pos-x) 0.05) (< (abs go-pos-y) 0.05) (< (abs go-pos-theta) 1.2))
                   (setq recog-flag t)
                 (progn
                   (let (go-pos-x- go-pos-y- go-pos-theta-)
                     ;; clipping how robot moves because using :go-pos-`unsafe`
                     (setq go-pos-x- (min go-pos-x 0.1))
                     (setq go-pos-x- (max go-pos-x- -0.1))
                     (setq go-pos-y- (min go-pos-y 0.1))
                     (setq go-pos-y- (max go-pos-y- -0.1))
                     (setq go-pos-theta- (min go-pos-theta 3))
                     (setq go-pos-theta- (max go-pos-theta- -3))

                     (send *ri* :go-pos-unsafe go-pos-x- go-pos-y- go-pos-theta-)
>>>>>>> 1d91b1e3... formatting by emacs
                     )
                   )
=======
     (while (null recog-flag)
       (send *microwave* :move-to (send *ri* :state :worldcoords) :world)
       (send *pr2* :move-to (send *ri* :state :worldcoords) :world)

       (send *pr2* :head :neck-p :joint-angle 28)
       (send *ri* :angle-vector (send *pr2* :angle-vector))
       (send *ri* :wait-interpolation)

       (setq cds (check-detection :type "microwave" :speak-name "denshirenji" :timeout 10 :speak nil))
       (send *microwave* :move-to (make-coords :pos (send cds :pos)))
       (send *microwave* :translate (float-vector 115 450 -220))  ;; x奥行き y横 z高さ
       (setq microwave-pos (send *microwave* :pos))
       (setq microwave-rpy (send *microwave* :rpy-angle))

       (let* ((local-microwave-x (elt (send cds :worldpos) 0))
              (local-microwave-y (elt (send cds :worldpos) 1))
              (local-microwave-theta (car (car (rpy-angle (send cds :worldrot)))))
              (local-microwave-without-z (make-coords :pos (float-vector local-microwave-x local-microwave-y 0) :rpy (list local-microwave-theta 0 0)))
              (world-microwave-without-z ;; pose of microwave in world when z = 0
               (send (send (send *pr2* :worldcoords) :copy-worldcoords) :transform local-microwave-without-z))
              (relative-coords
               (make-coords :pos (float-vector 600 0 0)))
              (reach-world-microwave-without-z ;; pose in front of microwave in world when z = 0
               (send (send world-microwave-without-z :copy-worldcoords) :transform relative-coords))
              (reach-local-microwave-local-without-z
               (send (send (send *pr2* :worldcoords) :copy-worldcoords)
                     :transformation reach-world-microwave-without-z))
              )
         (send world-microwave-without-z :draw-on :flush t :size 200 :width 5 :color (float-vector 1 0 0))
         (send reach-world-microwave-without-z :draw-on :flush t :size 200 :width 5 :color (float-vector 0 1 0))
         (let* ((go-pos-x (* 1e-3 (elt (send reach-local-microwave-local-without-z :worldpos) 0)))
                (go-pos-y (* 1e-3 (elt (send reach-local-microwave-local-without-z :worldpos) 1)))
                (go-pos-theta (+ (rad2deg (car (car (rpy-angle (send reach-local-microwave-local-without-z :worldrot))))) 180))
                )
           (if (> go-pos-theta 180)
               (setq go-pos-theta (- go-pos-theta 360)))
           (if (< go-pos-theta -180)
               (setq go-pos-theta + go-pos-theta 360))

           (format t "go-pos x:~a y:~a theta:~a~%"
                   go-pos-x go-pos-y go-pos-theta)
           (if (and (< (abs go-pos-x) 0.05) (< (abs go-pos-y) 0.05) (< (abs go-pos-theta) 1.2))
               (setq recog-flag t)
             (progn
               (let (go-pos-x- go-pos-y- go-pos-theta-)
                 ;; clipping how robot moves because using :go-pos-`unsafe`
                 (setq go-pos-x- (min go-pos-x 0.1))
                 (setq go-pos-x- (max go-pos-x- -0.1))
                 (setq go-pos-y- (min go-pos-y 0.1))
                 (setq go-pos-y- (max go-pos-y- -0.1))
                 (setq go-pos-theta- (min go-pos-theta 3))
                 (setq go-pos-theta- (max go-pos-theta- -3))

                 (send *ri* :go-pos-unsafe go-pos-x- go-pos-y- go-pos-theta-)
>>>>>>> c4c50c75... organize euslisp code
                 )
               )
             )
           )
         )
       )
     (send *ri* :go-pos-unsafe -0.15 0 0)
     )
   ;;KO move arm back into previous position
   (send *pr2* :rarm :angle-vector #f(-39.4936 26.0339 -93.5104 -120.297 -203.777 -58.4382 258.91))
   (when use-ri
       (send *ri* :angle-vector (send *pr2* :angle-vector) 1500)
       (send *ri* :wait-interpolation))
   )

  (:update-microwave-pose
   ()
   (let ((local-cds)
         (microwave-cds (make-coords)))
     (send *microwave* :move-to (send *ri* :state :worldcoords) :world)
     (send *pr2* :move-to (send *ri* :state :worldcoords) :world)

     (send *pr2* :head :neck-p :joint-angle 28)
     (send *ri* :angle-vector (send *pr2* :angle-vector))
     (send *ri* :wait-interpolation)

     (setq local-cds (check-detection :type "microwave" :speak-name "denshirenji" :timeout 10 :speak nil))
     (send microwave-cds :move-to *pr2* :world)
     (send microwave-cds :transform local-cds)
     (send *microwave* :move-to microwave-cds :world)
     (setq *test* (send *microwave* :copy-worldcoords))   ;; TODO delete

     (send *microwave* :rotate pi/2 :y)
     (send *microwave* :rotate pi :x)
     (send *microwave* :translate (float-vector 0 370 -300))  ;; -220

     (setq microwave-pos (send *microwave* :pos))
     (setq microwave-rpy (send *microwave* :rpy-angle))
     )
   )

  (:open-microwave
   (&optional (wait? nil))
   (send *ri* :stop-grasp :larm)
   (send *ri* :wait-interpolation)

   (send *microwave* :move-to (make-coords :pos microwave-pos :rpy (car microwave-rpy)) :world)
   (send *microwave* :angle-vector (float-vector 180))

<<<<<<< HEAD
   (let* ((handle-coords))
     (pr2-left-reset-pose)   ;; (send *pr2* :reset-pose)

     ;; pre-open1
<<<<<<< HEAD
     (setq handle-coords (make-coords :pos (send m-handle :worldpos) :rpy (float-vector (car (car (rpy-angle (send m-handle :worldrot)))) 0 0)))
     (send handle-coords :translate (float-vector 0 -30 -50))
     (send *pr2* :larm :inverse-kinematics (send (send handle-coords :copy-worldcoords) :rotate (/ pi -2) :y)
           :rotation-axis t)
     (if wait? (wait-enter))
     (when use-ri
       (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
       (send *ri* :wait-interpolation))


     ;; pre-open2
     (setq handle-coords (make-coords :pos (send m-handle :worldpos) :rpy (float-vector (car (car (rpy-angle (send m-handle :worldrot)))) 0 0)))
     (send handle-coords :translate (float-vector 0 0 50))
=======
     (setq handle-coords (make-coords :pos (send m-handle :worldpos) :rpy (float-vector (car (car (rpy-angle (send m-handle :worldrot)))) 0 0)))
     (send handle-coords :translate (float-vector 0 -30 -50))
>>>>>>> 3f3ed6c5... some implementation
     (send *pr2* :larm :inverse-kinematics (send (send handle-coords :copy-worldcoords) :rotate (/ pi -2) :y)
           :rotation-axis t)
     (if wait? (wait-enter))
     (when use-ri
=======
   (let ((handle-coords))
     (pr2-left-reset-pose)

     (progn   ;; pre-open1
       (setq handle-coords (make-coords :pos (send (send m-handle :copy-worldcoords) :worldpos) :rpy (float-vector (car (car (rpy-angle (send (send m-handle :copy-worldcoords) :worldrot)))) 0 0)))
       (send handle-coords :translate (float-vector -80 -10 -50))
       (send *pr2* :larm :inverse-kinematics (send (send handle-coords :copy-worldcoords) :rotate (/ pi -2) :y) :rotation-axis t)
       (if wait? (wait-enter))
>>>>>>> c4c50c75... organize euslisp code
       (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
       (send *ri* :wait-interpolation)
       )

     (progn   ;; pre-open2
       (setq handle-coords (make-coords :pos (send (send m-handle :copy-worldcoords) :worldpos) :rpy (float-vector (car (car (rpy-angle (send (send m-handle :copy-worldcoords) :worldrot)))) 0 0)))
       (send handle-coords :translate (float-vector -80 -10 50))
       (send *pr2* :larm :inverse-kinematics (send (send handle-coords :copy-worldcoords) :rotate (/ pi -2) :y) :rotation-axis t)
       (if wait? (wait-enter))
       (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
       (send *ri* :wait-interpolation)
       )

<<<<<<< HEAD
<<<<<<< HEAD
=======
     ;; pre-open2
     (setq handle-coords (make-coords :pos (send m-handle :worldpos) :rpy (float-vector (car (car (rpy-angle (send m-handle :worldrot)))) 0 0)))
     (send handle-coords :translate (float-vector 0 0 50))
     (send *pr2* :larm :inverse-kinematics (send (send handle-coords :copy-worldcoords) :rotate (/ pi -2) :y)
           :rotation-axis t)
     (if wait? (wait-enter))
     (when use-ri
       (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
       (send *ri* :wait-interpolation))


>>>>>>> 3f3ed6c5... some implementation
     ;; open
     (let ((i 170)
           av
=======
     (let ((i 170)   ;; open
           (av)
>>>>>>> c4c50c75... organize euslisp code
           (tms (list))
           (avs (list)))
       (while (> i 109)
         (send *microwave* :angle-vector (float-vector i))
         (setq handle-coords (make-coords :pos (send (send m-handle :copy-worldcoords) :worldpos) :rpy (float-vector (car (car (rpy-angle (send (send m-handle :copy-worldcoords) :worldrot)))) 0 0)))
         (send handle-coords :translate (float-vector -80 -10 50))

         (send *pr2* :larm :inverse-kinematics (send (send handle-coords :copy-worldcoords) :rotate (/ pi -2) :y) :rotation-axis t)
         (setq av (send *pr2* :angle-vector))
         (setq avs (append avs (list av)))
         (setq tms (append tms (list 1000)))

         (setq i (- i 5))
         )
       (send *ri* :angle-vector-sequence avs tms)
       (send *ri* :wait-interpolation)
       )

     (progn   ;; detach left hand from microwave
       (send *pr2* :larm :inverse-kinematics (send (send handle-coords :copy-worldcoords) :rotate (/ pi -2) :y) :rotation-axis t)
       (if wait? (wait-enter))
       (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
       (send *ri* :wait-interpolation)

       (send *pr2* :larm :inverse-kinematics (send (send (send handle-coords :copy-worldcoords) :rotate (/ pi -2) :y) :translate (float-vector -50 0 60)) :rotation-axis t)
       (if wait? (wait-enter))
       (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
       (send *ri* :wait-interpolation)

       (send *pr2* :larm :inverse-kinematics (send (send (send handle-coords :copy-worldcoords) :rotate (/ pi -2) :y) :translate (float-vector -50 -10 60)) :rotation-axis t)
       (if wait? (wait-enter))
       (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
       (send *ri* :wait-interpolation)

       (send *pr2* :larm :inverse-kinematics (send (send (send handle-coords :copy-worldcoords) :rotate (/ pi -2) :y) :translate (float-vector -200 0 10)) :rotation-axis t)
       (if wait? (wait-enter))
       (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
       (send *ri* :wait-interpolation)

       (pr2-left-reset-pose)
       (if wait? (wait-enter))
       (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
       (send *ri* :wait-interpolation)
       )
     )
   )

  (:close-microwave
   (&optional (wait? nil))
   (let* ((handle-coords))
     (pr2-left-reset-pose)
     (send *microwave* :angle-vector (float-vector 70))

     (progn   ;; pre-hang left hand to microwave
       (setq handle-coords (make-coords :pos (send (send (send m-handle :copy-worldcoords) :translate (float-vector -100 -100 -50)) :worldpos) :rpy (float-vector (car (car (rpy-angle (send m-handle :worldrot)))) 0 0)))
       (send *pr2* :larm :inverse-kinematics (send (send handle-coords :copy-worldcoords) :rotate (/ pi -2) :y) :rotation-axis nil)
       (if wait? (wait-enter))
       (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
       (send *ri* :wait-interpolation)
       )

     (progn   ;; hang left hand to microwave
       (setq handle-coords (make-coords :pos (send (send (send m-handle :copy-worldcoords) :translate (float-vector -100 -100 130)) :worldpos) :rpy (float-vector (car (car (rpy-angle (send m-handle :worldrot)))) 0 0)))
       (send *pr2* :larm :inverse-kinematics (send (send handle-coords :copy-worldcoords) :rotate (/ pi -2) :y) :rotation-axis nil)
       (if wait? (wait-enter))
       (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
       (send *ri* :wait-interpolation)
       )

     (let ((i 110)   ;; close
           (av)
           (tms (list))
           (avs (list)))
       (while (< i 216)
         (send *microwave* :angle-vector (float-vector i))
         (setq handle-coords (make-coords :pos (send (send (send m-handle :copy-worldcoords) :translate (float-vector -100 -100 130)) :worldpos)
                                          :rpy (float-vector (car (car (rpy-angle (send m-handle :worldrot)))) 0 0)))
         (send *pr2* :larm :inverse-kinematics (send (send handle-coords :copy-worldcoords) :rotate (/ pi -2) :y) :rotation-axis nil)
         (setq av (send *pr2* :angle-vector))
         (setq avs (append avs (list av)))
         (setq tms (append tms (list 400)))
         (setq i (+ i 10))
         )
       (send *ri* :angle-vector-sequence avs tms)
       (send *ri* :wait-interpolation)
       )
     )

   (pr2-left-reset-pose)
   (if wait? (wait-enter))
   (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
   (send *ri* :wait-interpolation)
   )

<<<<<<< HEAD
<<<<<<< HEAD
  (:put-cup-handle 
   ()
   (send *pr2* :rarm :angle-vector #f(-6.62506 -3.96955 -41.286 -121.526 -151.42 -91.9334 236.595))
   (when use-ri (send *ri* :angle-vector (send *pr2* :angle-vector) 1500)
	 (send *ri* :wait-interpolation))
   (send *pr2* :rarm :angle-vector #f(4.8101 10.4514 -23.4935 -95.4876 -158.274 -72.0419 203.178))
   (when use-ri (send *ri* :angle-vector (send *pr2* :angle-vector) 1500)
	 (send *ri* :wait-interpolation))
   (send *pr2* :rarm :angle-vector #f(20.6674 -13.5168 -27.0534 -14.591 -136.61 -23.4884 171.545))
   (when use-ri (send *ri* :angle-vector (send *pr2* :angle-vector) 1500)
	 (send *ri* :wait-interpolation))
   ;;change lowerposition due to workaround for spinning
   ;;(send *pr2* :rarm :angle-vector #f(17.61 -5.36846 -32.4696 -26.94 -143.034 -24.1605 184.046))
   ;;(send *pr2* :rarm :angle-vector #f(17.2599 -7.17866 -36.8263 -25.4638 -139.507 -23.7161 176.153))
   ;;change to pick up better
   (send *pr2* :rarm :angle-vector #f(15.1359 -7.46127 -43.6881 -32.3907 -137.927 -31.4548 183.473))
   (when use-ri (send *ri* :angle-vector (send *pr2* :angle-vector) 1500)
	 (send *ri* :wait-interpolation))
   (when use-ri (send *ri* :stop-grasp :rarm)
	 (send *ri* :wait-interpolation))
   ;;move back to initial pos
   (send *pr2* :rarm :angle-vector #f(20.6674 -13.5168 -27.0534 -14.591 -136.61 -23.4884 171.545))
   (when use-ri (send *ri* :angle-vector (send *pr2* :angle-vector) 1500)
	 (send *ri* :wait-interpolation))
   (send *pr2* :rarm :angle-vector #f(4.8101 10.4514 -23.4935 -95.4876 -158.274 -72.0419 203.178))
   (when use-ri (send *ri* :angle-vector (send *pr2* :angle-vector) 1500)
	 (send *ri* :wait-interpolation))
   (send *pr2* :rarm :angle-vector #f(-6.62506 -3.96955 -41.286 -121.526 -151.42 -91.9334 236.595))
   (when use-ri (send *ri* :angle-vector (send *pr2* :angle-vector) 1500)
	 (send *ri* :wait-interpolation))
   (send *pr2* :rarm :angle-vector #f(-39.4936 26.0339 -93.5104 -120.297 -203.777 -58.4382 258.91))
   (when use-ri (send *ri* :angle-vector (send *pr2* :angle-vector) 1500)
	 (send *ri* :wait-interpolation))
   (print "put cup with handle into microwave"))

  
  (:take-cup-handle 
   ()
   (send *pr2* :rarm :angle-vector #f(-6.62506 -3.96955 -41.286 -121.526 -151.42 -91.9334 236.595))
   (when use-ri (send *ri* :angle-vector (send *pr2* :angle-vector) 1500)
	 (send *ri* :wait-interpolation))
   (send *pr2* :rarm :angle-vector #f(4.8101 10.4514 -23.4935 -95.4876 -158.274 -72.0419 203.178))
   (when use-ri (send *ri* :angle-vector (send *pr2* :angle-vector) 1500)
	 (send *ri* :wait-interpolation))
   (send *pr2* :rarm :angle-vector #f(20.6674 -13.5168 -27.0534 -14.591 -136.61 -23.4884 171.545))
   (when use-ri (send *ri* :angle-vector (send *pr2* :angle-vector) 1500)
	 (send *ri* :wait-interpolation))
   ;;(send *pr2* :rarm :angle-vector #f(17.61 -5.36846 -32.4696 -26.94 -143.034 -24.1605 184.046))
   ;;(send *pr2* :rarm :angle-vector #f(17.2599 -7.17866 -36.8263 -25.4638 -139.507 -23.7161 176.153))
   (send *pr2* :rarm :angle-vector #f(15.1359 -7.46127 -43.6881 -32.3907 -137.927 -31.4548 183.473))
   (when use-ri (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
	 (send *ri* :wait-interpolation))
   ;;move towards object slowly
   (send *pr2* :rarm :angle-vector #f(21.5466 -13.8811 -34.5959 -8.59437 -122.618 -23.8449 155.107))
   (when use-ri (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
	 (send *ri* :wait-interpolation))
   ;;close gripper
   (when use-ri (send *ri* :start-grasp :rarm :gain 0.05)
	 (send *ri* :wait-interpolation))
   ;;move back to initial pos
   (send *pr2* :rarm :angle-vector #f(20.6674 -13.5168 -27.0534 -14.591 -136.61 -23.4884 171.545))
   (when use-ri (send *ri* :angle-vector (send *pr2* :angle-vector) 1500)
	 (send *ri* :wait-interpolation))
   (send *pr2* :rarm :angle-vector #f(4.8101 10.4514 -23.4935 -95.4876 -158.274 -72.0419 203.178))
   (when use-ri (send *ri* :angle-vector (send *pr2* :angle-vector) 1500)
	 (send *ri* :wait-interpolation))
   (send *pr2* :rarm :angle-vector #f(-6.62506 -3.96955 -41.286 -121.526 -151.42 -91.9334 236.595))
   (when use-ri (send *ri* :angle-vector (send *pr2* :angle-vector) 1500)
	 (send *ri* :wait-interpolation))
   (send *pr2* :rarm :angle-vector #f(-39.4936 26.0339 -93.5104 -120.297 -203.777 -58.4382 258.91))
   (when use-ri (send *ri* :angle-vector (send *pr2* :angle-vector) 1500)
	 (send *ri* :wait-interpolation))
   (print "put cup with handle into microwave"))
  
=======
  ;; TODO
>>>>>>> c4c50c75... organize euslisp code
=======
>>>>>>> f9184cc9... suceeded in open, put object and close
  (:put-object
   (&optional (wait? nil))
   (send *ri* :start-grasp :rarm)
   (send *ri* :wait-interpolation)

   (send *microwave* :move-to (make-coords :pos microwave-pos :rpy (car microwave-rpy)) :world)
   (send *microwave* :angle-vector (float-vector 100))

<<<<<<< HEAD
<<<<<<< HEAD
   ;; pre-put
   (send *pr2* :rarm :angle-vector #f(-23.6954 65.1019 -97.3693 -118.161 118.624 -85.6695 -112.431))
   (if wait? (wait-enter))
   (when use-ri (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
         (send *ri* :wait-interpolation))
   ;; (let ((current-coords (send *pr2* :rarm :end-coords :copy-worldcoords))
   ;;       (target-coords (send (send *microwave* :copy-worldcoords) :translate #f(-200 -200 150)))
   ;;       mid-coords)
   ;;   (dotimes (i 10)
   ;;     (setq mid-coords (make-coords :pos (midpoint (/ i 10.0) (send current-coords :worldpos) (send target-coords :worldpos))))
   ;;     (send *pr2* :rarm :inverse-kinematics
   ;;           (send mid-coords :rotate -pi/2 :z))
   ;;     ;;:rotation-axis :z)
   ;;     (if wait? (wait-enter))
   ;;     (when use-ri (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
   ;;           (send *ri* :wait-interpolation))
   ;;     )
   ;;   )
<<<<<<< HEAD
   (send *pr2* :rarm :inverse-kinematics
         (send (send (send *microwave* :copy-worldcoords) :translate #f(-200 -300 120)) :rotate 0 :x)
         :rotation-axis t)
   (if wait? (wait-enter))
   (when use-ri (send *ri* :angle-vector (send *pr2* :angle-vector) 6000)
         (send *ri* :wait-interpolation))

   ;; put
   (send *pr2* :rarm :inverse-kinematics
         (send (send (send *microwave* :copy-worldcoords) :translate #f(70 -300 120)) :rotate 0 :x)
=======
   (send *pr2* :rarm :inverse-kinematics
         (send (send (send *microwave* :copy-worldcoords) :translate #f(-200 -300 120)) :rotate 0 :x)
         :rotation-axis t)
   (if wait? (wait-enter))
   (when use-ri (send *ri* :angle-vector (send *pr2* :angle-vector) 6000)
         (send *ri* :wait-interpolation))

   ;; put
   (send *pr2* :rarm :inverse-kinematics
         (send (send (send *microwave* :copy-worldcoords) :translate #f(70 -300 120)) :rotate 0 :x)
         :rotation-axis t)
   (if wait? (wait-enter))
   (when use-ri (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
         (send *ri* :wait-interpolation))

   (send *pr2* :rarm :inverse-kinematics
         (send (send (send *microwave* :copy-worldcoords) :translate #f(70 -300 100)) :rotate 0 :x)
         :rotation-axis t)
   (if wait? (wait-enter))
   (when use-ri (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
         (send *ri* :wait-interpolation))

   (send *ri* :stop-grasp :rarm)
   (if wait? (wait-enter))
   (when use-ri (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
         (send *ri* :wait-interpolation))

   (send *pr2* :rarm :inverse-kinematics
         (send (send (send *microwave* :copy-worldcoords) :translate #f(70 -300 100)) :rotate 0 :x)
         :rotation-axis t)
   (if wait? (wait-enter))
   (when use-ri (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
         (send *ri* :wait-interpolation))

   ;; post-put
   (send *pr2* :rarm :inverse-kinematics
         (send (send (send *microwave* :copy-worldcoords) :translate #f(-200 -300 150)) :rotate 0 :x)
>>>>>>> 3f3ed6c5... some implementation
         :rotation-axis t)
   (if wait? (wait-enter))
   (when use-ri (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
         (send *ri* :wait-interpolation))

<<<<<<< HEAD
   (send *pr2* :rarm :inverse-kinematics
         (send (send (send *microwave* :copy-worldcoords) :translate #f(70 -300 100)) :rotate 0 :x)
         :rotation-axis t)
   (if wait? (wait-enter))
   (when use-ri (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
         (send *ri* :wait-interpolation))
=======
   (progn   ;; pre-put
     (send *pr2* :rarm :angle-vector #f(-23.6954 65.1019 -97.3693 -118.161 118.624 -85.6695 -112.431))
=======
   (progn   ;; pre-put1
     (send *pr2* :rarm :angle-vector (float-vector -23.6954 65.1019 -97.3693 -118.161 118.624 -85.6695 -112.431))
>>>>>>> f9184cc9... suceeded in open, put object and close
     (if wait? (wait-enter))
     (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
     (send *ri* :wait-interpolation)
     )

   (progn   ;; pre-put2
     (send *pr2* :rarm :inverse-kinematics (send (send (send *microwave* :copy-worldcoords) :translate (float-vector -100 -240 160)) :rotate 0 :x) :rotation-axis t)
     (if wait? (wait-enter))
     (send *ri* :angle-vector (send *pr2* :angle-vector) 6000)
     (send *ri* :wait-interpolation)
     )

   (progn   ;; put
     (send *pr2* :rarm :inverse-kinematics (send (send (send *microwave* :copy-worldcoords) :translate (float-vector 150 -240 160)) :rotate 0 :x) :rotation-axis t)
     (if wait? (wait-enter))
     (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
     (send *ri* :wait-interpolation)
>>>>>>> c4c50c75... organize euslisp code

     (send *pr2* :rarm :inverse-kinematics (send (send (send *microwave* :copy-worldcoords) :translate (float-vector 150 -240 120)) :rotate 0 :x) :rotation-axis t)
     (if wait? (wait-enter))
     (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
     (send *ri* :wait-interpolation)

     (send *ri* :stop-grasp :rarm)
     (if wait? (wait-enter))
     (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
     (send *ri* :wait-interpolation)

     (send *pr2* :rarm :inverse-kinematics (send (send (send *microwave* :copy-worldcoords) :translate (float-vector 150 -240 120)) :rotate 0 :x) :rotation-axis t)
     (if wait? (wait-enter))
     (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
     (send *ri* :wait-interpolation)
     )

<<<<<<< HEAD
=======
>>>>>>> 3f3ed6c5... some implementation
   ;; post-put
   (send *pr2* :reset-pose)
   (if wait? (wait-enter))
   (when use-ri (send *ri* :angle-vector (send *pr2* :angle-vector) 6000)
         (send *ri* :wait-interpolation))
=======
   (progn   ;; post-put1
     (send *pr2* :rarm :inverse-kinematics (send (send (send *microwave* :copy-worldcoords) :translate (float-vector -100 -240 160)) :rotate 0 :x) :rotation-axis t)
     (if wait? (wait-enter))
     (send *ri* :angle-vector (send *pr2* :angle-vector) 6000)
     (send *ri* :wait-interpolation)
     )

   (progn   ;; post-put2
     (send *pr2* :reset-pose)
     (if wait? (wait-enter))
     (send *ri* :angle-vector (send *pr2* :angle-vector) 6000)
     (send *ri* :wait-interpolation)
     )
>>>>>>> c4c50c75... organize euslisp code
   )





  ;; TODO
  (:take-object
   (&optional (wait? nil))
   (send *ri* :stop-grasp :rarm)
   (send *ri* :wait-interpolation)

   (send *microwave* :move-to (make-coords :pos microwave-pos :rpy (car microwave-rpy)) :world)
   (send *microwave* :angle-vector (float-vector 100))

   (progn   ;; pre-put
     (send *pr2* :rarm :angle-vector (float-vector -23.6954 65.1019 -97.3693 -118.161 118.624 -85.6695 -112.431))
     (send *pr2* :rarm :inverse-kinematics (send (send (send *microwave* :copy-worldcoords) :translate (float-vector -200 -300 150)) :rotate 0 :x) :rotation-axis t)
     (if wait? (wait-enter))
     (send *ri* :angle-vector (send *pr2* :angle-vector) 6000)
     (send *ri* :wait-interpolation)
     )

   (progn   ;; put
     (send *pr2* :rarm :inverse-kinematics (send (send (send *microwave* :copy-worldcoords) :translate (float-vector 70 -300 100)) :rotate 0 :x) :rotation-axis t)
     (if wait? (wait-enter))
     (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
     (send *ri* :wait-interpolation)

     (send *ri* :start-grasp :rarm)
     (if wait? (wait-enter))
     (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
     (send *ri* :wait-interpolation)
     )

   (progn   ;; post-put1
     (send *pr2* :rarm :inverse-kinematics (send (send (send *microwave* :copy-worldcoords) :translate (float-vector -200 -300 150)) :rotate 0 :x) :rotation-axis t)
     (if wait? (wait-enter))
     (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
     (send *ri* :wait-interpolation)
     )

   (progn   ;; post-put2
     (pr2-left-reset-pose)
     (if wait? (wait-enter))
     (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
     (send *ri* :wait-interpolation)

     (send *pr2* :rarm :angle-vector (float-vector -23.6954 65.1019 -97.3693 -118.161 118.624 -85.6695 -112.431))
     (if wait? (wait-enter))
     (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
     (send *ri* :wait-interpolation)
     )
   )

  ;; TODO
  (:push-heat-button
   (&optional (wait? nil))
   (send *pr2* :reset-pose)
   (send *ri* :start-grasp :rarm)
   (send *ri* :wait-interpolation)

   (send *microwave* :move-to (make-coords :pos microwave-pos :rpy (car microwave-rpy)) :world)
   (send *microwave* :angle-vector (float-vector 180))

<<<<<<< HEAD
<<<<<<< HEAD
   (let ((heat-button-front (send (send *microwave* :copy-worldcoords) :transform (make-coords :pos (float-vector -70 -505 115))))
         (heat-button (send (send *microwave* :copy-worldcoords) :transform (make-coords :pos (float-vector 10 -505 115)))))
=======
   (let ((heat-button-front (send (send *microwave* :copy-worldcoords) :transform (make-coords :pos (float-vector -70 -480 170))))
         (heat-button (send (send *microwave* :copy-worldcoords) :transform (make-coords :pos (float-vector 0 -480 170)))))
>>>>>>> 3f3ed6c5... some implementation
=======
   (let ((heat-button-front (send (send *microwave* :copy-worldcoords) :transform (make-coords :pos (float-vector -70 -505 115))))
         (heat-button (send (send *microwave* :copy-worldcoords) :transform (make-coords :pos (float-vector 10 -505 115)))))
>>>>>>> c21fb06f... some implementation

<<<<<<< HEAD
     (send *pr2* :rarm :inverse-kinematics heat-button-front
           :rotation-axis t)
     (if wait? (wait-enter))
<<<<<<< HEAD
<<<<<<< HEAD
     (when use-ri (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
           (send *ri* :wait-interpolation))
     ;;(unix::sleep 1)
=======
     (when use-ri (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
           (send *ri* :wait-interpolation))
>>>>>>> 3f3ed6c5... some implementation
=======
     (when use-ri (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
           (send *ri* :wait-interpolation))
     (unix::sleep 1)
>>>>>>> c21fb06f... some implementation


     (send *pr2* :rarm :inverse-kinematics heat-button
           :rotation-axis t)
     (if wait? (wait-enter))
<<<<<<< HEAD
     (when use-ri (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
           (send *ri* :wait-interpolation))

     ;;KO add movement to left right up and down to touch button even if vision imperfect
     (setq *r-tcp* (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
     ;;one centimeter up
     (send *r-tcp* :translate (float-vector 0 0 10) :world)
     (send *pr2* :rarm :inverse-kinematics *r-tcp*)
     (when use-ri (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
           (send *ri* :wait-interpolation))
     ;;one centimeter down
     (send *r-tcp* :translate (float-vector 0 0 -20) :world)
     (when use-ri (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
           (send *ri* :wait-interpolation))
     ;;one centimeter left
     (send *r-tcp* :translate (float-vector 0 10 0) :world)
     (when use-ri (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
           (send *ri* :wait-interpolation))
     ;;one centimeter right
     (send *r-tcp* :translate (float-vector 0 -20 0) :world)
     (when use-ri (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
           (send *ri* :wait-interpolation))
     
=======
     (when use-ri (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
           (send *ri* :wait-interpolation))

>>>>>>> 3f3ed6c5... some implementation
     (send *pr2* :rarm :inverse-kinematics heat-button-front
           :rotation-axis t)
     (if wait? (wait-enter))
     (when use-ri (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
           (send *ri* :wait-interpolation))
=======
     (progn   ;; pre-push
       (send *pr2* :rarm :inverse-kinematics heat-button-front :rotation-axis t)
       (if wait? (wait-enter))
       (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
       (send *ri* :wait-interpolation)
       )

     (progn   ;; push
       (send *pr2* :rarm :inverse-kinematics heat-button :rotation-axis t)
       (if wait? (wait-enter))
       (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
       (send *ri* :wait-interpolation)
       )

     (progn   ;; post-push
       (send *pr2* :rarm :inverse-kinematics heat-button-front :rotation-axis t)
       (if wait? (wait-enter))
       (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
       (send *ri* :wait-interpolation)
       )
>>>>>>> c4c50c75... organize euslisp code
     )
   )

  ;; TODO
  (:push-start-button
   (&optional (wait? nil))
   (send *pr2* :reset-pose)
   (send *pr2* :start-grasp :rarm)
   (send *ri* :start-grasp :rarm)

   (send *microwave* :move-to (make-coords :pos microwave-pos :rpy (car microwave-rpy)) :world)
   (send *microwave* :angle-vector (float-vector 180))

<<<<<<< HEAD
<<<<<<< HEAD
   (let ((start-button-front (send (send *microwave* :copy-worldcoords) :transform (make-coords :pos (float-vector -70 -510 170))))
         (start-button (send (send *microwave* :copy-worldcoords) :transform (make-coords :pos (float-vector -5 -510 170)))))
=======
   (let ((start-button-front (send (send *microwave* :copy-worldcoords) :transform (make-coords :pos (float-vector -70 -480 170))))
         (start-button (send (send *microwave* :copy-worldcoords) :transform (make-coords :pos (float-vector -10 -480 170)))))
>>>>>>> 3f3ed6c5... some implementation
=======
   (let ((start-button-front (send (send *microwave* :copy-worldcoords) :transform (make-coords :pos (float-vector -70 -510 170))))
         (start-button (send (send *microwave* :copy-worldcoords) :transform (make-coords :pos (float-vector -5 -510 170)))))
>>>>>>> c21fb06f... some implementation

<<<<<<< HEAD
     (send *pr2* :rarm :inverse-kinematics start-button-front
           :rotation-axis t)
     (if wait? (wait-enter))
<<<<<<< HEAD
<<<<<<< HEAD
     (when use-ri (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
=======
     (when use-ri (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
>>>>>>> 3f3ed6c5... some implementation
=======
     (when use-ri (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
>>>>>>> c21fb06f... some implementation
           (send *ri* :wait-interpolation))

=======
     (progn ;; pre-push
       (send *pr2* :rarm :inverse-kinematics start-button-front :rotation-axis t)
       (if wait? (wait-enter))
       (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
       (send *ri* :wait-interpolation)
       )
>>>>>>> c4c50c75... organize euslisp code

     (progn   ;; push
       (send *pr2* :rarm :inverse-kinematics start-button :rotation-axis t)
       (if wait? (wait-enter))
       (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
       (send *ri* :wait-interpolation)
       )

     (progn   ;; post-push
       (send *pr2* :rarm :inverse-kinematics start-button-front :rotation-axis t)
       (if wait? (wait-enter))
       (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
       (send *ri* :wait-interpolation)
       )
     )

   (send *pr2* :reset-pose)
   (if wait? (wait-enter))
   (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
   (send *ri* :wait-interpolation)
   )
  )
<<<<<<< HEAD


(defun execute ()
<<<<<<< HEAD
  (setq *pmi* (instance pr2-microwave-interface :init t)) ;;added low torso instead of reset-pose

  ;;(send *pmi* :go-to-microwave-roughly)
  ;;need to change av, pr2 goes into reset pose
  ;;ADD AV to avoid hitting the cupboard with cup!!
  (send *pmi* :go-to-microwave-accurately)

  ;; put object in microwave
  ;;have tomove right arm collision left arm with cup!!!!
  (send *pmi* :open-microwave) ;;works even with my grippers and gloves!

  ;;(send *pmi* :go-forward)
  ;;(send *pmi* :put-object) ;;need to change for cup, careful with infinity joint
  (send *pmi* :put-cup-handle)

  ;;(send *pmi* :go-backward)
  (send *pmi* :close-microwave)

  ;; push button
  ;;(send *pmi* :go-to-microwave-accurately)
  (send *pmi* :push-heat-button) ;;needs tuning!!
  (send *pmi* :push-start-button) ;;needs tuning

  ;; wait for baking
  (unix::sleep (* 2 60))

  ;; take object from microwave
  (send *pmi* :open-microwave)
  (send *pmi* :take-cup-handle)
  ;;(send *pmi* :go-forward)
  ;;(send *pmi* :take-object) ;;carefu with infinity joint!

  ;;(send *pmi* :go-backward)
  (send *pmi* :close-microwave)
  )



  (defun start-tracking ()
    (let ((service-name "start_tracking"))
      (ros::wait-for-service service-name)      
      (ros::service-call service-name (instance std_srvs::EmptyRequest :init))))
  


(defun terminate-tracking ()
(let ((service-name "terminate_tracking"))
(ros::wait-for-service service-name)
(ros::service-call service-name (instance std_srvs::EmptyRequest :init))))

(defun execute-anne ()
  (setq *pmi* (instance pr2-microwave-interface :init t)) ;;added low torso instead of reset-pose
  (send *pmi* :go-to-microwave-accurately)
  (speak-en "I am sorry I was not able to find the microwave can you give me a second try with a fresh start")
  (send *pmi* :open-microwave)
  (send *pmi* :put-cup-handle)
  (send *pmi* :close-microwave)
  ;;(send *pmi* :push-heat-button) ;;needs tuning!!
  ;;(send *pmi* :push-start-button) ;;needs tuning
  ;; wait for baking
  (speak-en "I can not push the buttons yet I am sorry I still have a lot to learn could you help me this time")
  ;;(unix::sleep (* 2 60))
  ;; take object from microwave
  (send *pmi* :open-microwave)
  (send *pmi* :take-cup-handle)
  (send *pmi* :close-microwave)
  (send *ri* :go-pos-unsafe 0 0 120)
  (speak-en "who wants cake")
  )

(defun exec-anne-1st-part()
  (setq *pmi* (instance pr2-microwave-interface :init t)) ;;added low torso instead of reset-pose
  ;;(start-tracking)
  (send *pmi* :go-to-microwave-accurately)
  ;;(terminate-tracking)
  )

(defun exec-anne-2nd-part ()
  (send *pmi* :open-microwave)
  (send *pmi* :put-cup-handle)
  (send *pmi* :close-microwave)
  ;;(send *pmi* :push-heat-button) ;;needs tuning!!
  ;;(send *pmi* :push-start-button) ;;needs tuning
  ;; wait for baking
  (speak-en "I can not push the buttons yet I am sorry I still have a lot to learn could you help me this time")
  ;;(unix::sleep (* 2 60))
  (unix::sleep 135)
  ;; take object from microwave
  (send *pmi* :open-microwave)
  (send *pmi* :take-cup-handle)
  (send *pmi* :close-microwave)
  (send *ri* :go-pos-unsafe 0 0 180)
  (speak-en "who wants cake")
=======
  (setq *pmi* (instance pr2-microwave-interface :init t))

  ;;(send *pmi* :go-to-microwave-roughly)
  (send *pmi* :go-to-microwave-accurately)

  ;; put object in microwave
  (send *pmi* :open-microwave)

  (send *pmi* :go-forward)
  (send *pmi* :put-object)

  (send *pmi* :go-backward)
  (send *pmi* :close-microwave)

  ;; push button
  (send *pmi* :go-to-microwave-accurately)
  (send *pmi* :push-heat-button)
  (send *pmi* :push-start-button)

  ;; wait for baking
  (unix::sleep (* 2 60))

  ;; take object from microwave
  (send *pmi* :open-microwave)

  (send *pmi* :go-forward)
  (send *pmi* :take-object)

  (send *pmi* :go-backward)
  (send *pmi* :close-microwave)
>>>>>>> 3f3ed6c5... some implementation
  )
=======
>>>>>>> c4c50c75... organize euslisp code
