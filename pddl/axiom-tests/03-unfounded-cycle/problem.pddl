(define (problem ax03-p)
  (:domain ax03-cycle)
  ;; base is false -> least fixpoint gives p = q = false.
  (:init)
  (:goal (goal-reached)))

;; Correct plan: set-base, reach-goal
;;   set-base -> base -> p [via base] -> q -> reach-goal -> goal-reached.
;;
;; CRITICAL soundness check: at horizon 0 (no action), the goal must be
;; UNSOLVABLE. If the encoding accepts the unfounded p = q = true, reach-goal
;; fires with no action and the goal is spuriously reached at length 0.
