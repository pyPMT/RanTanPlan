(define (domain ax06-numeric)
  (:requirements :strips :numeric-fluents :derived-predicates)

  (:predicates
    (flyable)   ;; derived from a numeric guard
    (arrived))

  (:functions
    (fuel))

  ;; The body is a numeric comparison atom. It is Boolean-sorted: the propagator
  ;; watches (>= (fuel) 5) as an opaque truth-valued leaf; Z3 owns the arithmetic.
  (:derived (flyable) (>= (fuel) 5))

  (:action fly
    :precondition (flyable)
    :effect (and (arrived) (decrease (fuel) 5)))

  (:action refuel
    :precondition (and)
    :effect (increase (fuel) 10)))
