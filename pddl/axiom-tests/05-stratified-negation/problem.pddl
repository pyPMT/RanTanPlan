(define (problem ax05-p)
  (:domain ax05-strat)
  ;; obstacle true -> blocked true -> clear false at layer 0.
  (:init (obstacle))
  (:goal (moved)))

;; Correct plan: remove-obstacle, move
;;   remove-obstacle -> obstacle false -> blocked re-derives false ->
;;   clear re-derives true -> move -> moved.
