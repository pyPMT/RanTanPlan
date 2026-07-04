// Python extension module (_rantanplan).
//
// Interim contract (nanobind migration, Phase 3): the problem travels as
// protobuf bytes so the transport can be verified against the subprocess
// path before the contract itself changes. Phase 5 replaces it with the
// plain-data IR and the preprocess/solve two-function surface.

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <stdexcept>
#include <string>
#include <vector>

#include "unified_planning.pb.h"

#include "api/solver_api.hpp"
#include "config/config.hpp"
#include "config/strategy_registry.hpp"
#include "problem/protobuf_io.hpp"
#include "util/logger.hpp"
#include "util/stats.hpp"

#include "z3++.h"

namespace nb = nanobind;

namespace {

nb::bytes solve_bytes(nb::bytes problem_pb, const std::vector<std::string>& args) {
    // These flags exit(0) inside the CLI parser, which would kill the
    // interpreter; they have module-level equivalents instead.
    for (const auto& arg : args) {
        if (arg == "--help" || arg == "--list-strategies") {
            throw std::invalid_argument(
                arg + " is not usable in-process (use list_strategies())");
        }
    }

    // Rebuild the argv shape the CLI parser expects: <prog> <in> <out> [OPTIONS]
    std::vector<std::string> argv_storage = {"_rantanplan", "<in-process>", "<in-process>"};
    argv_storage.insert(argv_storage.end(), args.begin(), args.end());
    std::vector<char*> argv;
    argv.reserve(argv_storage.size());
    for (auto& arg : argv_storage) argv.push_back(arg.data());

    auto& config = rantanplan::Config::instance();
    config.initialize(static_cast<int>(argv.size()), argv.data());
    rantanplan::Logger::instance().set_verbosity(config.global.verbosity);

    // Match the CLI driver's Z3 verbosity mapping (SILENT=0, INFO=0, VERBOSE=1, DEBUG=5)
    constexpr int z3_verbosity[] = {0, 0, 1, 5};
    z3::set_param("verbose", z3_verbosity[static_cast<int>(config.global.verbosity)]);

    // Parse while holding the GIL (reads the Python bytes buffer)
    Problem problem_msg;
    if (!problem_msg.ParseFromArray(problem_pb.c_str(),
                                    static_cast<int>(problem_pb.size()))) {
        throw std::invalid_argument("could not parse the protobuf problem payload");
    }

    PlanGenerationResult result;
    {
        nb::gil_scoped_release release;

        rantanplan::Problem problem = rantanplan::load_problem_from_protobuf(problem_msg);
        const auto preprocessor_options = rantanplan::PreprocessorOptions::from_config(config);
        auto pre = rantanplan::api::preprocess(std::move(problem), preprocessor_options);

        if (pre.proven_unsolvable) {
            result.set_status(PlanGenerationResult_Status_UNSOLVABLE_PROVEN);
            auto* log_message = result.add_log_messages();
            log_message->set_level(LogMessage_LogLevel_INFO);
            log_message->set_message("Problem proven unsolvable: goals not reachable in " +
                                     pre.unsolvable_reason);
        } else {
            config.planner.start_timestep = pre.lower_bound;
            result = rantanplan::api::solve(pre);
        }

        // Mirror the CLI driver's post-solve stats behavior
        if (config.is_debug()) {
            rantanplan::Stats::instance().print_all();
        }
        if (!config.global.stats_file.empty()) {
            rantanplan::Stats::instance().write_to_file(config.global.stats_file);
        }
    }

    const std::string out = result.SerializeAsString();
    return nb::bytes(out.data(), out.size());
}

std::string z3_version() {
    unsigned major = 0, minor = 0, build = 0, revision = 0;
    Z3_get_version(&major, &minor, &build, &revision);
    return std::to_string(major) + "." + std::to_string(minor) + "." +
           std::to_string(build) + "." + std::to_string(revision);
}

} // namespace

NB_MODULE(_rantanplan, m) {
    m.doc() = "In-process RantanPlan solver (C++ core via nanobind)";

    m.def("solve", &solve_bytes, nb::arg("problem_pb"), nb::arg("args"),
          "Solve a problem serialized as unified_planning protobuf bytes with "
          "the main binary's CLI flags; returns PlanGenerationResult bytes. "
          "The GIL is released while solving.");

    m.def("list_strategies", &rantanplan::StrategyRegistry::list_strategies,
          "Names of all registered strategies.");

    m.def("z3_version", &z3_version,
          "Version of the libz3 this module is linked against (must match "
          "the z3-solver package's, or two copies are loaded).");
}
