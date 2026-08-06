// Contract test for the fork's THEORY_PLAN.
//
// Written at M3 as a throwaway; kept at M4 because it is the only regression
// test for the fork's public API surface, which M5-M7 change. It proves that:
//   1. PLAN_DOES / PLAN_FLUENT / PLAN_AUX entities can be built through the
//      public API and carry the sort declared in their payload,
//   2. their (index, timestep, sort) round-trips through Op::getIndices,
//   3. distinct indices give distinct atoms and equal indices give the same
//      atom (hash-consing on the operator payload),
//   4. the atoms reach the SAT solver as ordinary literals -- a satisfiable
//      formula over them is SAT and an unsatisfiable one is UNSAT,
//   5. THEORY_PLAN actually ran, evidenced by its statistics prefix,
//   6. the "PLAN" logic token parses, so a tight logic can still admit the
//      theory (plan §7.7 trap 2).
//
// Not part of `rantanplan_core` and nothing links it:
//   cmake --build <dir> --target cvc5_theory_plan_smoke
// See docs/cvc5/migration-plan.md §8 M3/M4.

#include <cvc5/cvc5.h>

#include <iostream>
#include <string>
#include <vector>

using namespace cvc5;

namespace {

int failures = 0;

void check(bool ok, const std::string& what)
{
  std::cout << (ok ? "  ok   " : "  FAIL ") << what << "\n";
  if (!ok) ++failures;
}

// does(a, t) -- action `a` executed at timestep `t`.
Term does(TermManager& tm, uint32_t a, uint32_t t)
{
  return tm.mkTerm(tm.mkOp(Kind::PLAN_DOES, {a, t}), {});
}

// holds(f, t) -- Boolean fluent `f` true in the state at timestep `t`.
Term holds(TermManager& tm, uint32_t f, uint32_t t)
{
  return tm.mkPlanFluent(f, t, tm.getBooleanSort());
}

// value(f, t) -- numeric fluent `f`'s value at timestep `t`. Same kind as
// holds(): role and sort are orthogonal, and only the payload differs.
Term value(TermManager& tm, uint32_t f, uint32_t t)
{
  return tm.mkPlanFluent(f, t, tm.getIntegerSort());
}

// ---------------------------------------------------------------------------
// 1. Term construction, typing, and index round-trip.
// ---------------------------------------------------------------------------
void test_atoms()
{
  std::cout << "[1] atom construction and identity\n";
  TermManager tm;

  Term a = does(tm, 7, 3);
  check(a.getSort().isBoolean(), "PLAN_DOES atom is Boolean-sorted");
  check(a.getKind() == Kind::PLAN_DOES, "kind survives construction");

  Op op = a.getOp();
  check(op.getNumIndices() == 2, "PLAN_DOES has 2 indices");
  check(op[0].getUInt32Value() == 7 && op[1].getUInt32Value() == 3,
        "indices round-trip as (index=7, timestep=3)");
  check(a.getSort().isBoolean(), "sort comes from Term::getSort(), not an index");

  // Hash-consing: identity is the (kind, index, timestep) triple and nothing
  // else. This is the property that replaces RantanPlan's string-keyed reverse
  // lookup (z3_variable_factory.cpp:159-175).
  check(does(tm, 7, 3) == a, "same (index, timestep) is the same atom");
  check(does(tm, 7, 4) != a, "different timestep is a different atom");
  check(does(tm, 8, 3) != a, "different index is a different atom");
  check(Term(holds(tm, 7, 3)) != a, "PLAN_FLUENT(7,3) differs from PLAN_DOES(7,3)");

  Term aux = tm.mkTerm(tm.mkOp(Kind::PLAN_AUX, {1, 0}), {});
  check(aux.getSort().isBoolean(), "PLAN_AUX atom is Boolean-sorted");

  // Role and sort are orthogonal: one kind, two sorts, distinct entities.
  Term n = value(tm, 7, 3);
  check(n.getSort().isInteger(), "PLAN_FLUENT with sort=Int is Integer-sorted");
  check(n.getKind() == Kind::PLAN_FLUENT, "...and is still the PLAN_FLUENT kind");
  check(n != holds(tm, 7, 3), "sort is part of identity: Int(7,3) != Bool(7,3)");

  // The payload carries a real Sort, so a fluent can range over any theory's
  // sort -- this is what makes planning *modulo theories* expressible. A
  // datatype-sorted fluent is the M4.5 target; check the mechanism works.
  DatatypeDecl colour = tm.mkDatatypeDecl("Colour");
  colour.addConstructor(tm.mkDatatypeConstructorDecl("red"));
  colour.addConstructor(tm.mkDatatypeConstructorDecl("white"));
  Sort colour_sort = tm.mkDatatypeSort(colour);
  Term hand = tm.mkPlanFluent(9, 0, colour_sort);
  check(hand.getSort() == colour_sort,
        "PLAN_FLUENT can carry a datatype sort (finite object universe)");
  check(hand.getKind() == Kind::PLAN_FLUENT, "...and is still the PLAN_FLUENT kind");

  std::cout << "      printed form: " << a << "\n";
}

// ---------------------------------------------------------------------------
// 2. The atoms behave as ordinary SAT literals.
// ---------------------------------------------------------------------------
void test_sat_unsat()
{
  std::cout << "[2] atoms reach the SAT solver\n";
  {
    TermManager tm;
    Solver solver(tm);
    solver.setOption("produce-models", "true");
    // does(0,0) AND (does(0,0) => holds(1,1))  -- satisfiable, and the
    // implication must propagate holds(1,1) to true in the model.
    Term d = does(tm, 0, 0);
    Term h = holds(tm, 1, 1);
    solver.assertFormula(d);
    solver.assertFormula(tm.mkTerm(Kind::IMPLIES, {d, h}));
    Result r = solver.checkSat();
    check(r.isSat(), "satisfiable formula over plan atoms is sat");
    if (r.isSat())
    {
      check(solver.getValue(h).getBooleanValue(),
            "implication propagated: holds(1,1) is true in the model");
    }
  }
  {
    TermManager tm;
    Solver solver(tm);
    // does(0,0) AND NOT does(0,0) -- unsat purely at the Boolean level.
    Term d = does(tm, 0, 0);
    solver.assertFormula(d);
    solver.assertFormula(tm.mkTerm(Kind::NOT, {d}));
    check(solver.checkSat().isUnsat(), "contradictory plan atoms are unsat");
  }
  {
    // A pigeonhole over plan atoms: 3 actions, 2 timesteps, at most one action
    // per timestep and every action must occur. UNSAT, and it exercises real
    // clause learning over PLAN_DOES literals rather than unit propagation.
    TermManager tm;
    Solver solver(tm);
    for (uint32_t a = 0; a < 3; ++a)
    {
      std::vector<Term> steps;
      for (uint32_t t = 0; t < 2; ++t) steps.push_back(does(tm, a, t));
      solver.assertFormula(tm.mkTerm(Kind::OR, steps));
    }
    for (uint32_t t = 0; t < 2; ++t)
      for (uint32_t a = 0; a < 3; ++a)
        for (uint32_t b = a + 1; b < 3; ++b)
          solver.assertFormula(tm.mkTerm(
              Kind::OR,
              {does(tm, a, t).notTerm(), does(tm, b, t).notTerm()}));
    check(solver.checkSat().isUnsat(), "3 actions into 2 timesteps is unsat");
  }
  {
    // A numeric fluent term under arithmetic: this is the case that only works
    // because PLAN_FLUENT is sort-carrying. It also exercises THEORY_PLAN
    // sharing a term with THEORY_ARITH (plan risk 3).
    TermManager tm;
    Solver solver(tm);
    solver.setOption("produce-models", "true");
    Term v0 = value(tm, 0, 0);
    Term v1 = value(tm, 0, 1);
    // v0 = 5  AND  v1 = v0 + 3
    solver.assertFormula(tm.mkTerm(Kind::EQUAL, {v0, tm.mkInteger(5)}));
    solver.assertFormula(tm.mkTerm(
        Kind::EQUAL, {v1, tm.mkTerm(Kind::ADD, {v0, tm.mkInteger(3)})}));
    Result r = solver.checkSat();
    check(r.isSat(), "numeric plan fluents under arithmetic are sat");
    if (r.isSat())
    {
      // getIntegerValue() returns a decimal string, not an integer.
      check(solver.getValue(v1).getIntegerValue() == "8",
            "arithmetic over PLAN_FLUENT evaluates: v1 = 8");
    }
    // ...and the same constraints made contradictory must be unsat.
    solver.assertFormula(tm.mkTerm(Kind::EQUAL, {v1, tm.mkInteger(9)}));
    check(solver.checkSat().isUnsat(), "contradictory numeric fluent is unsat");
  }
}

// ---------------------------------------------------------------------------
// 3. THEORY_PLAN was actually instantiated and stepped.
// ---------------------------------------------------------------------------
void test_theory_ran()
{
  std::cout << "[3] THEORY_PLAN is live\n";
  TermManager tm;
  Solver solver(tm);
  solver.assertFormula(does(tm, 0, 0));
  solver.checkSat();

  Statistics stats = solver.getStatistics();
  std::vector<std::string> seen;
  for (auto it = stats.begin(); it != stats.end(); ++it)
    if ((*it).first.rfind("theory::plan::", 0) == 0) seen.push_back((*it).first);

  // If "check" were missing from kinds.toml's properties list, the theory would
  // be registered but never dispatched to -- and this is what would catch it.
  check(!seen.empty(), "theory::plan::* statistics present");
  for (const auto& s : seen) std::cout << "      " << s << "\n";
}

// ---------------------------------------------------------------------------
// 4. The "PLAN" logic token (plan §7.7 trap 2).
// ---------------------------------------------------------------------------
void test_logic_token()
{
  std::cout << "[4] logic string admits the plan theory\n";
  {
    TermManager tm;
    Solver solver(tm);
    solver.setLogic("QF_PLAN");
    solver.assertFormula(does(tm, 0, 0));
    check(solver.checkSat().isSat(), "QF_PLAN accepts a plan atom");
  }
  {
    // The point of the logic_info.cpp hunks: a *tight* logic that still admits
    // the theory. Without them, setLogic("QF_LIA") would disable THEORY_PLAN
    // and the first plan atom would throw a LogicException.
    TermManager tm;
    Solver solver(tm);
    solver.setLogic("QF_PLANLIA");
    solver.assertFormula(does(tm, 0, 0));
    check(solver.checkSat().isSat(), "QF_PLANLIA accepts a plan atom");
  }
}

}  // namespace

int main()
{
  {
    TermManager tm;
    Solver s(tm);
    std::cout << "THEORY_PLAN smoke test -- cvc5 " << s.getVersion() << "\n\n";
  }
  test_atoms();
  test_sat_unsat();
  test_theory_ran();
  test_logic_token();
  std::cout << "\n" << (failures == 0 ? "ALL OK" : "FAILURES: ")
            << (failures == 0 ? "" : std::to_string(failures)) << "\n";
  return failures == 0 ? 0 : 1;
}
