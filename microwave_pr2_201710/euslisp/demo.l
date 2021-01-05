(require :pr2-microwave-interface "package://microwave_pr2_201710/euslisp/lib/pr2-microwave-interface.l")

(defun demo
    ()
  ;; init
  (setq *pmi* (instance pr2-microwave-interface :init))

  ;; go to microwave
  (send *pmi* :go-to-microwave-accurately)
  (send *pmi* :update-microwave-pose)

  ;; put object in microwave
  (send *pmi* :open-microwave)
  (send *pmi* :move-object t)
  (send *pmi* :close-microwave)

  ;; push button
  (send *ri* :angle-vector (send *pr2* :reset-pose) 2000)
  (send *pmi* :push-500w-button)
  (send *pmi* :push-1min-button)
  (send *pmi* :push-start-button)
  (send *ri* :angle-vector (send *pr2* :reset-pose) 2000)

  ;; wait for heating
  (unix::sleep (* 3 60))

  ;; take object from microwave
  (send *pmi* :open-microwave)
  (send *pmi* :move-object nil)
  (send *pmi* :close-microwave)
  )

(warn "~%(setq *pmi* (instance pr2-microwave-interface :init))~%(send *pmi* :go-to-microwave-accurately)~%")