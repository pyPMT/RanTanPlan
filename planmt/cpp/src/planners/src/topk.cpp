
#include <utility>
#include "planners/headers/topk.h"


namespace planmt {

    TopKPlanner::TopKPlanner(const Problem& problem, BaseEncoder& encoder, DiversePlannerCfg &cfg):
    DiversePlanner(problem, encoder, cfg)
    { }

    std::vector<Plan> TopKPlanner::search(size_t k, float upperbound) {
        std::vector<std::pair<Plan, Behaviour>> ret_plans;
        
        // First infer the formula length.
        auto formula_ = construct_formula();
        
        std::vector<std::unique_ptr<DiversityDimension>> dims;
        auto behaviour_space_dummy = BehaviourSpace(problem_, encoder_, dims);
        
        auto solver = create_solver_from(formula_);
        auto result = solver.check();
        
        if (result != z3::sat) {return std::vector<Plan>();}
        
        // Get the initial plan to infer the optimal plan lenght?
        ret_plans.push_back(std::make_pair(extract_plan(solver), Behaviour()));
        
        // Compute upperbound formula length.
        size_t upper_bound = static_cast<size_t>(formula_.length * upperbound);
        
        // Generate the remaning plans, and increment the formula in case there are
        // no plans available in the current plan lenght.
        for(size_t i = formula_.length; i < upper_bound; i++) {
            std::cout << "Check i: " << i << std::endl;
            auto generated_plans = produce_plans(formula_, k-ret_plans.size(), behaviour_space_dummy, ret_plans, ForbiddingMode::PLAN);
            ret_plans.insert(ret_plans.end(), generated_plans.begin(), generated_plans.end());
            if (ret_plans.size() >= k) { break; }
            increment_formula(formula_, i);
        }
        
        std::vector<Plan> all_generated_plans;
        for (auto &p: ret_plans) {
            all_generated_plans.push_back(p.first);
        }
        
        return all_generated_plans;
    }
}
