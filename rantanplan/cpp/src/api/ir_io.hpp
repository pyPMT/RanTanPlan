#pragma once

#include "ir.hpp"

namespace rantanplan {

class Problem;
class Plan;

/// Build a domain Problem from the plain-data IR (single forward interning
/// pass over the flat node pool).
Problem load_problem_from_ir(const ir::ProblemIR& ir_problem);

/// Serialize a domain Plan into IR action instances.
std::vector<ir::ActionInstanceIR> plan_to_ir(const Plan& plan);

} // namespace rantanplan
