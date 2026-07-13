(define (problem ax02-p)
  (:domain ax02-reach)
  (:objects a b c - node)
  ;; Only a->b exists; reach(a,c) is false at layer 0 (least fixpoint).
  (:init (edge a b))
  (:goal (reach a c)))

;; Correct plan: connect(b,c)
;;   edge(b,c) -> reach(b,c) [base] -> reach(a,c) [inductive, via a-b-c].
;;
;; Variant to also test by hand: goal (reach a c) with init empty must be
;; UNSOLVABLE at every horizon -- the least fixpoint never over-derives reach.
