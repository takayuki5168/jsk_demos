(load "package://jsk_2020_04_pr2_curry/euslisp/cook-with-pos-map/tool-use-codes.l")

(warn "please run (set-tools) ~%")

(defun set-tools ()
  (load "package://jsk_2020_04_pr2_curry/euslisp/cook-with-pos-map/tool-use-codes.l")
  (update-pos)
  
  (set-spatula-1)
  (set-ladle-1)

  (warn "please run (spatula-test) ~%")
  )

(defun spatula-test ()
  (load "package://jsk_2020_04_pr2_curry/euslisp/cook-with-pos-map/ih-use-codes.l")
  (update-pos)

  (spatula-mix-test)
  (put-spatula-1)

  (warn "please run (ladle-test) ~%")
  )

(defun ladle-test ()
  (load "package://jsk_2020_04_pr2_curry/euslisp/cook-with-pos-map/ih-use-codes.l")
  (update-pos)

  (ladle-mix-test)
  (put-ladle-1)  
  )


