%;; 初期化
(if (not (boundp '*pr2*))
    (progn
      (load "package://pr2eus/pr2-interface.l")
      (pr2-init)
      ))

(objects (list *pr2*))

(warn "please run (potato-peel-all) ~%")

(defun potato-peel-all ()
  ;; ピーラーをつかむ
  (load "tool-change/grasp-peeler.l")
  (grasp-peeler)

  ;; じゃがいもの皮を剥く．
  (load "rotate-peel-test/potato-peel-rotate-with-rec-tmp-with-sense-test-two.l")
  (now-peel)

  ;; じゃがいもを回転させる．
  (load "rotate-peel-test/carrot-rotate-with-rec-tmp.l")
  (now-rotate)

  ;; じゃがいもの皮を剥く．
  (load "rotate-peel-test/potato-peel-rotate-with-rec-tmp-with-sense-test-two.l")
  (now-peel)

  (warn "please run (potato-cut-all) ~%")
  )

(defun potato-cut-all ()
  ;; ピーラーから包丁に持ち替える．
  (load "tool-change/peeler-to-knife.l")
  (grasp-peel)

  ;; じゃがいもを半分に切る．
  (load "cut-stable/potato-cut-half-with-rec.l")
  (now-cut)

  ;; 右半分を立てる．
  (load "cut-stable/potato-half-right-set-with-rec.l")
  (now-set)
  
  ;; 容器に移す．
  (load "transfer-vegs/transfer-test-with-rec.l")
  (now-transfer)

  (warn "please run (carrot-demo) ~%")
  )

(defun carrot-demo ()
  
  )
