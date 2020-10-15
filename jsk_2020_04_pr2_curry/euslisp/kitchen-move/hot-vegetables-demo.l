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
  (load "tool-change/grasp-potato-peeler.l")
  (grasp-potato-peeler)

  ;; じゃがいもの皮を剥く．
  (load "rotate-peel-test/potato-peel-rotate-with-rec-tmp-with-sense-test-two.l")
  (now-peel)

  ;; じゃがいもを回転させる．
  (load "rotate-peel-test/potato-rotate-with-rec-tmp.l")
  (now-rotate)

  ;; じゃがいもの皮を剥く．
  (load "rotate-peel-test/potato-peel-rotate-with-rec-tmp-with-sense-test-two.l")
  (now-peel)

  (warn "please run (potato-cut-all) ~%")
  )

(defun potato-cut-all ()
  ;; ピーラーから包丁に持ち替える．
  (load "tool-change/peeler-to-knife.l")
  (put-potato-peeler)
  (grasp-peel)

  ;; じゃがいもを半分に切る．
  (load "cut-stable/potato-cut-half-with-rec.l")
  (now-cut)

  ;; 右半分を立てる．
  (load "cut-stable/potato-half-right-set-with-rec.l")
  (now-set)

  ;; 右半分を回転させる．
  (load "cut-stable/potato-half-right-rotate-with-rec.l")
  (now-rotate)

  ;; 右半分を切る．
  (load "cut-stable/potato.l")
  (now-rotate)

  ;; 包丁を置く
  (load "tool-change/put-knife.l")
  (put-knife)

  ;; 容器に移す．
  (load "transfer-vegs/transfer-test-with-rec.l")
  (now-transfer)

  (warn "please run (carrot-demo) ~%")
  )

(defun carrot-demo ()
  
  )
