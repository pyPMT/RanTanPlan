#pragma once

#include <cvc5/cvc5.h>

#include <vector>

namespace rantanplan {

/// The SMT term type. One alias so the ~600 solver-facing sites stay
/// backend-neutral and a future backend change is a one-line edit here.
using Term = cvc5::Term;

/// Term container. cvc5's API takes plain std::vector<Term> everywhere, so
/// this is pure shorthand -- no context, no conversions.
using TermVec = std::vector<cvc5::Term>;

}  // namespace rantanplan
