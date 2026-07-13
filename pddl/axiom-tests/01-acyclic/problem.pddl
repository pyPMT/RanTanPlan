(define (problem ax01-p)
  (:domain ax01-acyclic)
  ;; raw is unset -> ready derives false at layer 0.
  (:init)
  (:goal (done)))

;; Correct plan: prepare, finish
;;   prepare -> raw becomes true -> ready derives true -> finish -> done.
