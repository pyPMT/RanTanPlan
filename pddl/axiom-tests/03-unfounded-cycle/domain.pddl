(define (domain ax03-cycle)
  (:requirements :strips :disjunctive-preconditions :derived-predicates)

  (:predicates
    (base)          ;; basic: the only possible *foundation* for p
    (p)             ;; derived
    (q)             ;; derived
    (goal-reached))

  ;; p and q support each other in a cycle, with base as the only external
  ;; foundation. Least-fixpoint semantics: without base, p = q = false.
  ;; A naive head<->body completion would ALSO admit the unfounded model
  ;; p = q = true -- which this case exists to reject.
  (:derived (p) (or (base) (q)))
  (:derived (q) (p))

  (:action set-base
    :precondition (and)
    :effect (base))

  (:action reach-goal
    :precondition (p)
    :effect (goal-reached)))
