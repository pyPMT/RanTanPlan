(define (domain ax01-acyclic)
  (:requirements :strips :derived-predicates)

  (:predicates
    (raw)     ;; basic fluent, set by an action
    (ready)   ;; derived: non-recursively defined from (raw)
    (done))

  ;; Non-recursive axiom: ready holds exactly when raw holds.
  ;; This stratum is acyclic -> handled by eager completion (ready <-> raw).
  (:derived (ready) (raw))

  (:action prepare
    :precondition (and)
    :effect (raw))

  ;; Uses the derived predicate as a precondition.
  (:action finish
    :precondition (ready)
    :effect (done)))
