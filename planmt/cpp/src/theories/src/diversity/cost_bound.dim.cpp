
#include <numeric>


#include "theories/headers/diversity/cost_bound_dim.h"


namespace planmt {

CostBoundDimension::CostBoundDimension(const Problem& problem, BaseEncoder& encoder, float cost_bound_factor):
DiversityDimension(problem, encoder, std::string("cost-bound")),
ctx_(encoder_.get_variable_factory().get_context()),
cost_bound_vars_(ctx_),
accumlated_cost_literal_(ctx_){
    cost_bound_factor_ = cost_bound_factor;
    
    // Initialise encoders and extractors.
    // TODO: Check all encoders and decide which step_encode is required.
    // TODO: Joan, I guess we need a better thing than a name, I was thinking of encoder type like
    // the one used to create an encoder.
    auto _seq = [this](size_t t) { return this->sequnetial_step_encode(t); };
    encoders[std::string("SequentialSemantics")] = _seq;
    encoders[std::string("ExistsSemantics")]     = _seq;
}

std::vector<z3::expr> CostBoundDimension::encode(size_t timestep) {
    std::vector<z3::expr> ret_encodings;
    return encoders[encoder_.get_parallelism_strategy_name()](timestep);
}

dimension_value_t CostBoundDimension::extract(const z3::model & model) {
    auto v = model.eval(accumlated_cost_literal_, true);
    return dimension_value_t(std::string("cost-bound"), v.get_numeral_int(), accumlated_cost_literal_ == v);
}

std::vector<z3::expr> CostBoundDimension::sequnetial_step_encode(size_t timestep) {
    
    if (timestep < 1) {
        return std::vector<z3::expr>();
    }
    
    auto make_actions_vector = [](auto& encoder, int t, z3::context& ctx) {
        z3::expr_vector actions(ctx);
        for (auto& a : encoder.get_variable_factory().get_all_action_variables(t)) {
            actions.push_back(a);
        }
        return actions;
    };
    
    std::vector<z3::expr> ret_exprs;
    auto &var_factory = encoder_.get_variable_factory();
    z3::expr cost_literal = var_factory.create_int_variable("cost_at_" + std::to_string(timestep));
    
    auto actions_step = make_actions_vector(encoder_, timestep - 1, ctx_);
    ret_exprs.push_back(z3::implies(z3::mk_or(actions_step),  cost_literal == ctx_.int_val(1)));
    ret_exprs.push_back(z3::implies(!z3::mk_or(actions_step), cost_literal == ctx_.int_val(0)));
    
    cost_bound_vars_.push_back(cost_literal);
    if (cost_bound_vars_.size() > 1) {
        accumlated_cost_literal_ = var_factory.create_int_variable("acc_cost_at_" + std::to_string(timestep));
        ret_exprs.push_back(accumlated_cost_literal_ == z3::sum(cost_bound_vars_));
    }
    
    return ret_exprs;
}

std::vector<z3::expr> CostBoundDimension::parallel_step_encode(size_t timestep) {
    std::cout << std::endl;
}




}
