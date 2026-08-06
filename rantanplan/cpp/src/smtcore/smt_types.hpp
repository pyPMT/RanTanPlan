#pragma once

#include <cvc5/cvc5.h>

#include <vector>

namespace rantanplan {

/// The SMT term type. One alias so the ~600 former `z3::expr` sites read the
/// same as before and a future backend change is a one-line edit here.
using Term = cvc5::Term;

/// cvc5 has no container class -- `z3::expr_vector` was context-bound and had to
/// be constructed from a context; a plain vector needs nothing.
using TermVec = std::vector<cvc5::Term>;

}  // namespace rantanplan
