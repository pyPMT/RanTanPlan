(define (domain ax02-reach)
  (:requirements :strips :typing :existential-preconditions :derived-predicates)

  (:types node)

  (:predicates
    (edge ?x ?y - node)    ;; basic
    (reach ?x ?y - node))  ;; derived: recursive transitive closure

  ;; Two axioms for the same head (disjunctive definition):
  ;; base case ...
  (:derived (reach ?x ?y - node)
    (edge ?x ?y))
  ;; ... and inductive case (RECURSION: reach in its own body).
  ;; This SCC is size > 1 -> handled lazily by the propagator.
  (:derived (reach ?x ?y - node)
    (exists (?z - node) (and (edge ?x ?z) (reach ?z ?y))))

  (:action connect
    :parameters (?x ?y - node)
    :precondition (and)
    :effect (edge ?x ?y)))
