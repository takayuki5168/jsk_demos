#!/usr/bin/env roseus

;; load pr2-interface.l
(load "package://pr2eus/pr2-interface.l")

;; init 
(pr2-init)


;; reset-manip-pose
(send *pr2* :reset-manip-pose)
(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
(send *ri* :wait-interpolation)


;; トピックなどを定義
(defvar *topic-name* "/icp_registration/output_pose")


;; ros init
(ros::roseus "icp_tf_check" :anonymous t)
(ros::load-ros-manifest "geometry_msgs")
(setq *l* (instance ros::transform-listener :init))
(setq found-obj nil)
(setq obj-pos #f(0 0 0))
(setq obj-coord (make-coords :pos #f(0 0 0)))
(print 0)

;; define cb
(defun object-pose-cb (msg)
  (setq *msg* msg)
  (print 1)
  (when msg
    (setq tm (send *l* :get-latest-common-time "head_mount_kinect_rgb_optical_frame" "reference"))
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    (let* ((b (send msg :pose))
	   (ref->obj-coords (ros::tf-pose->coords b))
	   (cam->ref (send *l* :lookup-transform "head_mount_kinect_rgb_optical_frame" "reference" tm))
	   (cam-coords (send (send *pr2* :head_mount_kinect_rgb_optical_frame_lk) :copy-worldcoords)))
      (setq obj-coord (send (send cam-coords :transform cam->ref) :transform ref->obj-coords))
      (setq obj-pos (scale 0.001 (send obj-coord :worldpos)))
      (setq found-obj t)
      (print (list 'found? found-obj obj-pos))
      obj-pos)))


;; subscribeして認識
(ros::subscribe *topic-name* geometry_msgs::PoseStamped #'object-pose-cb 1)
(setq *b* (instance ros::transform-broadcaster :init))

(print 3)
(load "models/kettle-object.l")
(setq *kettle* (kattle))
(send *kettle* :locate #f(500 0 1000) :world)
(load "models/arrow-object.l")
(setq *axis* (arrow))
(send *arrow* :move-to (send *kettle* :handle-handle0) :world)
(objects (list *pr2* *kettle* *axis*))
(setq found-obj nil)
(until found-obj
  (print 4)
  (when (> (norm obj-pos) 0)
    (send *kettle* :move-to (scale 1000 obj-coord) :world);;;)
    (send *irtviewer* :draw-objects)

    ;;Rvizに表示をする。
    (setq tm (ros::time-now))
    (send *b* :send-transform obj-coord "base_footprint" "kettle" tm)

    )
  ;;
  (x::window-main-one) ;; IRT viewerの視点を変えられる。見にくければ変えよう
  (ros::spin-once)
  (ros::sleep)
  )


;; IKを解く
(send *axis* :move-to (send *kettle* :handle-handle0) :world)
(send *axis* :locate #f(-100 0 0))
(send *irtviewer* :draw-objects)

(send *pr2* :larm :inverse-kinematics
      *axis*
      :rotation-axis :t
      :debug-view t)
(send *ri* :angle-vector (send *pr2* :angle-vector))
(send *ri* :wait-interpolation)

(send *axis* :locate #f(100 0 0))
(send *irtviewer* :draw-objects)

(send *pr2* :larm :inverse-kinematics
      *axis*
      :rotation-axis :t
      :debug-view t)
(send *ri* :angle-vector (send *pr2* :angle-vector))
(send *ri* :wait-interpolation)

