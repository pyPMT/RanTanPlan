#pragma once

#include <z3++.h>

#include "problem/headers/problem.h"
#include "encoders/headers/grounded_encoder.h"


namespace planmt {


/**
 * @brief Reified grounded encoder for planning problems
 * 
 * TODO: Implement reification logic for CNF preconditions and goals.
 */
class ReifiedGroundedEncoder : public GroundedEncoder {
public:
    // Constructor
    ReifiedGroundedEncoder(const Problem& problem, z3::context& ctx);

    // TODO: Override encoding methods to add reification
    std::shared_ptr<z3::expr> encode_actions(size_t t) override;
    std::shared_ptr<z3::expr> encode_goal(size_t t) override;
    
private:
    // TODO: Add reification data structures and methods
};

} // namespace planmt
