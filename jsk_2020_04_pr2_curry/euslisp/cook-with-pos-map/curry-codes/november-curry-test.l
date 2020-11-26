(warn "please run (curry-0) ~%")

(defun curry-0 () ;; 非加熱調理の準備
  ;; ボウルをセットする
  (load "package://jsk_2020_04_pr2_curry/euslisp/cook-with-pos-map/bowl-use-codes.l")
  (update-pos)
  (now-set-1)
  (now-set-2)
  
  ;; 道具３つをセットする
  (load "package://jsk_2020_04_pr2_curry/euslisp/cook-with-pos-map/tool-use-codes.l")
  (update-pos)
  (now-set-tools) ;; 包丁，ピーラー，じゃがいもピーラーをセットする．
  (reset-rarm)
  
  (warn "please run (curry-1) ~%")
  )

;; 人参を洗ってセットする

(defun curry-1 () ;; 人参の下ごしらえ
  ;; ピーラーを掴む
  (load "package://jsk_2020_04_pr2_curry/euslisp/cook-with-pos-map/tool-use-codes.l")
  (update-pos)
  (grasp-peeler)
  
  ;; 人参皮剥き
  (load "package://")
  
  ;; ピーラーを置く，包丁を掴む
  (load "package://jsk_2020_04_pr2_curry/euslisp/cook-with-pos-map/tool-use-codes.l")
  (update-pos)
  (put-peeler)
  (grasp-knife)
  
  ;; 人参切断
  (load "package://")
  
  ;; 包丁を置く
  (load "package://jsk_2020_04_pr2_curry/euslisp/cook-with-pos-map/tool-use-codes.l")
  (update-pos)
  (put-knife)
  
  (warn "じゃがいもを洗ってセットして please run (curry-2) ~%")
  )

(defun curry-2 () ;; じゃがいもの下ごしらえ
  ;; じゃがいもピーラーを掴む
  (load "package://jsk_2020_04_pr2_curry/euslisp/cook-with-pos-map/tool-use-codes.l")
  (update-pos)
  (grasp-potato-peeler)
  
  ;; じゃがいもを剥く
  (load "package://")
  
  
  ;; じゃがいもピーラーを置いて包丁を掴む
  (load "package://jsk_2020_04_pr2_curry/euslisp/cook-with-pos-map/tool-use-codes.l")
  (update-pos)
  (put-potato-peeler)
  (grasp-knife)

  ;; じゃがいも切断
  (load "package://")
  
  
  ;; 包丁を置く
  (load "package://jsk_2020_04_pr2_curry/euslisp/cook-with-pos-map/tool-use-codes.l")
  (update-pos)
  (put-knife)
  
  (warn " 加熱準備の準備（棚を開けられたら無くせる）をして please run (curry-3) ~%")
  )

(defun curry-3 () ;; 加熱調理の準備
  ;; ボウルをセットする
  (load "package://jsk_2020_04_pr2_curry/euslisp/cook-with-pos-map/bowl-use-codes.l")
  (update-pos)
  (switch-bowl-2) ;; ボウル２をまな板の上に
  (now-set-3) ;; ボウル３をセットする
  (now-set-4) ;; ボウル４をセットする

  ;; ヘラとおたまを置く 
  (load "package://jsk_2020_04_pr2_curry/euslisp/cook-with-pos-map/tool-use-codes.l")
  (update-pos)
  (set-spatula-1)
  (set-ladle-1)
  
  (warn "玉ねぎをセットして　please run (curry-4) ~%")
  )

(defun curry-4 () ;; 玉ねぎを炒める
  ;; ヘラを掴む
  (load "package://jsk_2020_04_pr2_curry/euslisp/cook-with-pos-map/tool-use-codes.l")
  (update-pos)
  (grasp-spatula)
  
  ;; ボウル２を鍋に移す
  (load "package://jsk_2020_04_pr2_curry/euslisp/cook-with-pos-map/bowl-use-codes.l")
  (update-pos)
  (now-transfer-2)
  
  ;; コンロ操作をする
  
  
  ;; かき混ぜる
  
  
  (warn " 玉ねぎを炒めて確認し，肉をセットして please run (curry-5) ~%")
  )

(defun curry-5 () ;; 肉を炒める
  ;; ボウル３を鍋に移す
  (load "package://jsk_2020_04_pr2_curry/euslisp/cook-with-pos-map/bowl-use-codes.l")
  (update-pos)
  (now-transfer-3)
  
  ;; 
  
  (warn "肉を炒めて確認して please run (curry-6) ~%")
  )

(defun curry-6 () ;; 野菜を煮込む
  ;;
  
  (warn "please run (curry-7) ~%")
  )

(defun curry-7 () ;; カレールーを入れる
  ;;

  )
