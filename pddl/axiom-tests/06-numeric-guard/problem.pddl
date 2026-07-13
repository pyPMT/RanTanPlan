(define (problem ax06-p)
  (:domain ax06-numeric)
  ;; fuel = 3 < 5 -> flyable derives false at layer 0.
  (:init (= (fuel) 3))
  (:goal (arrived)))

;; Correct plan: refuel, fly
;;   refuel -> fuel = 13 -> (>= fuel 5) true -> flyable derives true ->
;;   fly -> arrived (fuel = 8).
;;
;; Bug catch: if the guard atom (>= fuel 5) is not materialized/watched, flyable
;; never derives and the goal is unreachable.
