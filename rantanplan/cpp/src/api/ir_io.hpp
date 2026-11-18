#pragma once

#include "ir.hpp"
#include "../problem/expr_enums.hpp"

namespace rantanplan {

class Problem;
class Plan;

/// Build a domain Problem from the plain-data IR (single forward interning
/// pass over the flat node pool).
Problem load_problem_from_ir(const ir::ProblemIR& ir_problem);

/// The IR kind of a planner node kind, the inverse of the loader's mapping (for
/// bindings that report pool nodes in the converter's terms).
ir::ExprKindIR to_ir_kind(ExprKind kind);

/// Serialize a domain Plan into IR action instances.
std::vector<ir::ActionInstanceIR> plan_to_ir(const Plan& plan);

} // namespace rantanplan
