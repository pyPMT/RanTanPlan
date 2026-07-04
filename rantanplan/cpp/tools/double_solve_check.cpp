// Cross-run global-state regression harness (nanobind migration, Phase 2).
//
// Runs two full preprocess+solve cycles in one process, re-initializing
// Config between them. Each run's plan must match a fresh single run of the
// same instance with the same options — any divergence means per-run state
// leaked across solves (Config re-initialization, Stats reset, hidden
// statics).
//
// Usage: double_solve_check <in1.pb> <out1.pb> <in2.pb> <out2.pb> [OPTIONS]
// OPTIONS are the main binary's flags and apply to both runs.

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "unified_planning.pb.h"

#include "api/solver_api.hpp"
#include "config/config.hpp"
#include "problem/protobuf_io.hpp"
#include "util/logger.hpp"

namespace {

int run_once(const char* prog, const std::string& input_file,
             const std::string& output_file,
             const std::vector<std::string>& options) {
    // Rebuild the argv shape the CLI parser expects: <prog> <in> <out> [OPTIONS]
    std::vector<std::string> args = {prog, input_file, output_file};
    args.insert(args.end(), options.begin(), options.end());
    std::vector<char*> argv;
    argv.reserve(args.size());
    for (auto& arg : args) argv.push_back(arg.data());

    auto& config = rantanplan::Config::instance();
    config.initialize(static_cast<int>(argv.size()), argv.data());
    rantanplan::Logger::instance().set_verbosity(config.global.verbosity);

    Problem problem_msg;
    std::fstream input(input_file, std::ios::in | std::ios::binary);
    if (!input || !problem_msg.ParseFromIstream(&input)) {
        std::cerr << "Could not open or parse problem file: " << input_file << std::endl;
        return 1;
    }
    rantanplan::Problem problem = rantanplan::load_problem_from_protobuf(problem_msg);

    const auto preprocessor_options = rantanplan::PreprocessorOptions::from_config(config);
    auto pre = rantanplan::api::preprocess(std::move(problem), preprocessor_options);

    PlanGenerationResult result;
    if (pre.proven_unsolvable) {
        result.set_status(PlanGenerationResult_Status_UNSOLVABLE_PROVEN);
    } else {
        config.planner.start_timestep = pre.lower_bound;
        result = rantanplan::api::solve(pre);
    }

    std::fstream output(output_file, std::ios::out | std::ios::trunc | std::ios::binary);
    if (!output || !result.SerializeToOstream(&output)) {
        std::cerr << "Could not write result to file: " << output_file << std::endl;
        return 1;
    }
    return 0;
}

} // namespace

int main(int argc, char* argv[]) {
    GOOGLE_PROTOBUF_VERIFY_VERSION;

    if (argc < 5) {
        std::cerr << "Usage: " << argv[0]
                  << " <in1.pb> <out1.pb> <in2.pb> <out2.pb> [OPTIONS]" << std::endl;
        return 1;
    }
    const std::vector<std::string> options(argv + 5, argv + argc);

    try {
        if (int rc = run_once(argv[0], argv[1], argv[2], options)) return rc;
        if (int rc = run_once(argv[0], argv[3], argv[4], options)) return rc;
    } catch (const std::exception& e) {
        std::cerr << "double_solve_check failed: " << e.what() << std::endl;
        return 1;
    }

    google::protobuf::ShutdownProtobufLibrary();
    return 0;
}
