(define (domain ordered-counters)
 (:requirements :strips :numeric-fluents)
 (:functions (value_x) (value_y))
 (:action inc_x
  :parameters ()
  :precondition (and (<= (value_x) 10))
  :effect (and (increase (value_x) 1)))
 (:action inc_y
  :parameters ()
  :precondition (and (<= (value_y) 10))
  :effect (and (increase (value_y) 1)))
)
