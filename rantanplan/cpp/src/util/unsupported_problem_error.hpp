#pragma once

#include <stdexcept>

namespace rantanplan {

/// The problem asks for something the planner does not handle: a language
/// feature it does not implement (a whole-array operation, a mixed add/remove
/// chain on a set, a state-dependent index in a goal, ...), or something the
/// chosen strategy would get wrong (declared state invariants under a strategy
/// that cannot keep them).
///
/// Passes, grounding and strategy resolution throw it; the API reports it as
/// the status UNSUPPORTED_PROBLEM with this message, where any other exception
/// is an internal error. (The Python module raises it as
/// _rantanplan.UnsupportedProblemError, a ValueError.) It derives from
/// std::invalid_argument, which these sites threw before.
class UnsupportedProblemError : public std::invalid_argument {
public:
    using std::invalid_argument::invalid_argument;
};

} // namespace rantanplan
