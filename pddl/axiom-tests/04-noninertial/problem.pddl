(define (problem ax04-p)
  (:domain ax04-noninertial)
  ;; fire true -> hot derives true at layer 0.
  (:init (fire))
  (:goal (safe)))

;; Correct plan: extinguish, declare-safe
;;   extinguish -> fire false -> hot MUST re-derive false (no frame axiom
;;   pins it) -> declare-safe -> safe.
;;
;; Bug catch: if hot gets a frame axiom, it persists true after extinguish and
;; declare-safe never becomes applicable -> unsolvable.
