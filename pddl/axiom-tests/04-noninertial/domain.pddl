(define (domain ax04-noninertial)
  (:requirements :strips :negative-preconditions :derived-predicates)

  (:predicates
    (fire)   ;; basic
    (hot)    ;; derived from fire
    (safe))

  ;; hot is true exactly while fire is true. hot is NON-INERTIAL: it must be
  ;; recomputed every layer. No action ever touches hot directly.
  (:derived (hot) (fire))

  (:action extinguish
    :precondition (fire)
    :effect (not (fire)))

  ;; Needs the derived predicate to be FALSE.
  (:action declare-safe
    :precondition (not (hot))
    :effect (safe)))
