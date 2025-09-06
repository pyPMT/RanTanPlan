

#include "planners/headers/topk.h"


namespace planmt {

    TopKPlanner::TopKPlanner(const Problem& problem, BaseEncoder& encoder, z3::context& ctx, DiversePlannerCfg &cfg):
    DiversePlanner(problem, encoder, ctx, cfg, std::vector<DiversityDimension>())
    { }

    std::vector<Plan> TopKPlanner::search(size_t k, float upperbound) {
        std::vector<Plan> ret_plans;
        
        // First infer the formula length.
        construct_formula();
        
        // Get the initial plan to infer the optimal plan lenght?
        ret_plans.push_back(extract_plan(formula_length));
        
        // Compute upperbound formula length.
        size_t upper_bound = static_cast<size_t>(formula_length * upperbound);
        
        // Generate the remaning plans, and increment the formula in case there are
        // no plans available in the current plan lenght.
        for(size_t i = formula_length; i < upper_bound; i++) {
            auto generated_plans = produce_plans(k-ret_plans.size(), ret_plans, ForbiddingMode::PLAN);
            ret_plans.insert(ret_plans.end(), generated_plans.begin(), generated_plans.end());
            if (ret_plans.size() >= k) { break; }
            increment_formula();
        }
        
        return ret_plans;
    }
}
