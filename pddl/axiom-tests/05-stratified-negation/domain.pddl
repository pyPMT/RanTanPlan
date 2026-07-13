(define (domain ax05-strat)
  (:requirements :strips :negative-preconditions :derived-predicates)

  (:predicates
    (obstacle)   ;; basic
    (blocked)    ;; derived, stratum 1
    (clear)      ;; derived, stratum 2 (negates stratum 1)
    (moved))

  ;; Stratum 1: blocked <- obstacle.
  (:derived (blocked) (obstacle))
  ;; Stratum 2: clear <- not blocked. Negates a lower stratum, so strata must
  ;; be evaluated in order (blocked settled before clear).
  (:derived (clear) (not (blocked)))

  (:action remove-obstacle
    :precondition (obstacle)
    :effect (not (obstacle)))

  (:action move
    :precondition (clear)
    :effect (moved)))
