/**
 * @file bindings.cpp
 * @brief Python extension module (_rantanplan).
 *
 * The problem travels as the plain-data IR (api/ir.hpp), built by
 * rantanplan/converter.py; results come back as ResultIR. The module
 * mirrors the C++ two-layer api: preprocess() returns an opaque
 * Preprocessed handle that solve() consumes.
 */

#include <nanobind/nanobind.h>
#include <nanobind/stl/function.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/unique_ptr.h>
#include <nanobind/stl/vector.h>

#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "api/ir.hpp"
#include "api/ir_io.hpp"
#include "api/solver_api.hpp"
#include "config/config.hpp"
#include "config/strategy_registry.hpp"
#include "util/logger.hpp"
#include "util/stats.hpp"

#include "z3++.h"

namespace nb = nanobind;
using namespace rantanplan;

namespace {

/// (Re-)initialize the Config singleton from module args and sync the
/// logger / Z3 verbosity, exactly like the CLI driver did.
void initialize_config(const std::vector<std::string>& args) {
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

    auto& config = Config::instance();
    config.initialize(static_cast<int>(argv.size()), argv.data());
    Logger::instance().set_verbosity(config.global.verbosity);

    // Match the CLI driver's Z3 verbosity mapping (SILENT=0, INFO=0, VERBOSE=1, DEBUG=5)
    constexpr int z3_verbosity[] = {0, 0, 1, 5};
    z3::set_param("verbose", z3_verbosity[static_cast<int>(config.global.verbosity)]);
}

/// Log sink callback: (level: int, text: str), level = VerbosityLevel
/// (0 = error/always, 1 = info, 2 = verbose, 3 = debug).
using LogCallback = std::function<void(int, const std::string&)>;

/// Install (or clear, with None) a Python callable as the Logger sink.
/// Called with the GIL held, so dropping the previous sink (a Python object)
/// is safe. Not synchronized against concurrent emission: install/clear it
/// from the thread that runs preprocess()/solve() (the wrapper does).
void set_log_sink(const std::optional<LogCallback>& sink) {
    if (sink) {
        // nanobind's std::function wrapper re-acquires the GIL when the
        // callable is invoked, so logging from the solver thread is safe.
        LogCallback cb = *sink;
        Logger::instance().set_sink(
            [cb](VerbosityLevel level, const std::string& text) {
                cb(static_cast<int>(level), text);
            });
    } else {
        Logger::instance().set_sink({});
    }
}

/// Opaque handle owning the preprocessed problem between preprocess() and
/// solve(). Single-solve: artifacts are single-consumer (solve() moves
/// achievers/SDAC bounds out of the bundle), so a second solve() on the
/// same handle is rejected instead of silently running without artifacts.
struct Preprocessed {
    PreprocessedProblem pre;
    std::vector<std::string> args; // args preprocess() ran with; solve() reuses them by default
    api::InterruptSlot interrupt_slot; // cross-thread cancel for the running solve
    bool consumed = false;
};

/// The solve tail shared by solve() and solve_anytime(): unsolvable
/// short-circuit, horizon seeding, post-solve stats.
ir::ResultIR run_solve(PreprocessedProblem& pre, const api::PlanCallback& on_plan = {},
                       api::InterruptSlot* interrupt_slot = nullptr) {
    auto& config = Config::instance();

    ir::ResultIR result;
    if (pre.proven_unsolvable) {
        result.status = static_cast<int>(ir::SolveStatusIR::UNSOLVABLE_PROVEN);
        result.log_messages.push_back(ir::LogMessageIR{
            1 /* INFO */,
            "Problem proven unsolvable: goals not reachable in " + pre.unsolvable_reason});
    } else {
        // The pipeline's horizon lower bound drives where the search starts.
        config.planner.start_timestep = pre.lower_bound;
        result = api::solve(pre, on_plan, interrupt_slot);
    }

    if (config.is_debug()) {
        Stats::instance().print_all();
    }
    // Stats travel in the result; file writing (--stats-file) is the
    // Python side's job now.
    for (const auto& [key, value] : Stats::instance().all()) {
        result.stats.emplace_back(key, value);
    }
    return result;
}

std::unique_ptr<Preprocessed> preprocess_ir(const ir::ProblemIR& problem_ir,
                                            const std::vector<std::string>& args) {
    initialize_config(args);

    nb::gil_scoped_release release;
    auto handle = std::make_unique<Preprocessed>();
    handle->args = args;
    rantanplan::Problem problem = load_problem_from_ir(problem_ir);
    const auto options = PreprocessorOptions::from_config(Config::instance());
    handle->pre = api::preprocess(std::move(problem), options);
    return handle;
}

ir::ResultIR solve_handle_impl(Preprocessed& handle,
                               const std::optional<std::vector<std::string>>& args,
                               const api::PlanCallback& on_plan) {
    if (handle.consumed) {
        throw std::runtime_error(
            "this Preprocessed handle was already solved; preprocessing "
            "artifacts are single-consumer -- call preprocess() again");
    }
    // No args (the normal path) -> reuse the args the handle was preprocessed
    // with; an explicit list replaces them (expert-only: option-dependent
    // preprocessing decisions are already baked into the handle).
    initialize_config(args ? *args : handle.args);
    handle.consumed = true;

    nb::gil_scoped_release release;
    return run_solve(handle.pre, on_plan, &handle.interrupt_slot);
}

ir::ResultIR solve_handle(Preprocessed& handle,
                          const std::optional<std::vector<std::string>>& args) {
    return solve_handle_impl(handle, args, {});
}

ir::ResultIR solve_anytime_handle(
        Preprocessed& handle,
        const std::function<void(const std::vector<ir::ActionInstanceIR>&)>& on_plan,
        const std::optional<std::vector<std::string>>& args) {
    // nanobind's std::function wrapper re-acquires the GIL when the Python
    // callable is invoked, so calling it from the solver thread (GIL released
    // in solve_handle_impl) is safe.
    return solve_handle_impl(handle, args, [&on_plan](const Plan& plan) {
        on_plan(plan_to_ir(plan));
    });
}

std::string export_formula_ir(const ir::ProblemIR& problem_ir,
                              const std::vector<std::string>& args) {
    initialize_config(args);

    nb::gil_scoped_release release;
    rantanplan::Problem problem = load_problem_from_ir(problem_ir);
    return api::export_formula(problem);
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

    // ── Problem IR ──────────────────────────────────────────────────────
    nb::class_<ir::ExprNodeIR>(m, "ExprNodeIR")
        .def(nb::init<>())
        .def_rw("kind", &ir::ExprNodeIR::kind)
        .def_rw("type_name", &ir::ExprNodeIR::type_name)
        .def_rw("symbol", &ir::ExprNodeIR::symbol)
        .def_rw("int_value", &ir::ExprNodeIR::int_value)
        .def_rw("real_value", &ir::ExprNodeIR::real_value)
        .def_rw("bool_value", &ir::ExprNodeIR::bool_value)
        .def_rw("children", &ir::ExprNodeIR::children);

    nb::class_<ir::TypeIR>(m, "TypeIR")
        .def(nb::init<>())
        .def_rw("name", &ir::TypeIR::name)
        .def_rw("parent", &ir::TypeIR::parent);

    nb::class_<ir::ObjectIR>(m, "ObjectIR")
        .def(nb::init<>())
        .def_rw("name", &ir::ObjectIR::name)
        .def_rw("type_name", &ir::ObjectIR::type_name);

    nb::class_<ir::ParameterIR>(m, "ParameterIR")
        .def(nb::init<>())
        .def_rw("name", &ir::ParameterIR::name)
        .def_rw("type_name", &ir::ParameterIR::type_name);

    nb::class_<ir::FluentIR>(m, "FluentIR")
        .def(nb::init<>())
        .def_rw("name", &ir::FluentIR::name)
        .def_rw("value_type", &ir::FluentIR::value_type)
        .def_rw("parameters", &ir::FluentIR::parameters)
        .def_rw("lower_bound", &ir::FluentIR::lower_bound)
        .def_rw("upper_bound", &ir::FluentIR::upper_bound);

    nb::class_<ir::EffectIR>(m, "EffectIR")
        .def(nb::init<>())
        .def_rw("kind", &ir::EffectIR::kind)
        .def_rw("fluent", &ir::EffectIR::fluent)
        .def_rw("value", &ir::EffectIR::value)
        .def_rw("condition", &ir::EffectIR::condition)
        .def_rw("forall", &ir::EffectIR::forall);

    nb::class_<ir::ActionIR>(m, "ActionIR")
        .def(nb::init<>())
        .def_rw("name", &ir::ActionIR::name)
        .def_rw("parameters", &ir::ActionIR::parameters)
        .def_rw("precondition", &ir::ActionIR::precondition)
        .def_rw("effects", &ir::ActionIR::effects)
        .def_rw("cost", &ir::ActionIR::cost);

    nb::class_<ir::AssignmentIR>(m, "AssignmentIR")
        .def(nb::init<>())
        .def_rw("fluent", &ir::AssignmentIR::fluent)
        .def_rw("value", &ir::AssignmentIR::value);

    nb::class_<ir::ProblemIR>(m, "ProblemIR")
        .def(nb::init<>())
        .def_rw("nodes", &ir::ProblemIR::nodes)
        .def_rw("types", &ir::ProblemIR::types)
        .def_rw("objects", &ir::ProblemIR::objects)
        .def_rw("fluents", &ir::ProblemIR::fluents)
        .def_rw("actions", &ir::ProblemIR::actions)
        .def_rw("initial_state", &ir::ProblemIR::initial_state)
        .def_rw("goals", &ir::ProblemIR::goals)
        .def_rw("state_invariants", &ir::ProblemIR::state_invariants)
        .def_rw("metric_kind", &ir::ProblemIR::metric_kind);

    // ── Results ─────────────────────────────────────────────────────────
    nb::enum_<ir::SolveStatusIR>(m, "SolveStatus")
        .value("SOLVED_SATISFICING", ir::SolveStatusIR::SOLVED_SATISFICING)
        .value("SOLVED_OPTIMALLY", ir::SolveStatusIR::SOLVED_OPTIMALLY)
        .value("UNSOLVABLE_PROVEN", ir::SolveStatusIR::UNSOLVABLE_PROVEN)
        .value("UNSOLVABLE_INCOMPLETELY", ir::SolveStatusIR::UNSOLVABLE_INCOMPLETELY)
        .value("TIMEOUT", ir::SolveStatusIR::TIMEOUT);

    nb::class_<ir::ActionInstanceIR>(m, "ActionInstanceIR")
        .def_ro("action_name", &ir::ActionInstanceIR::action_name)
        .def_ro("parameters", &ir::ActionInstanceIR::parameters);

    nb::class_<ir::LogMessageIR>(m, "LogMessageIR")
        .def_ro("level", &ir::LogMessageIR::level)
        .def_ro("message", &ir::LogMessageIR::message);

    nb::class_<ir::ResultIR>(m, "ResultIR")
        .def_ro("status", &ir::ResultIR::status)
        .def_ro("has_plan", &ir::ResultIR::has_plan)
        .def_ro("plan", &ir::ResultIR::plan)
        .def_ro("log_messages", &ir::ResultIR::log_messages)
        .def_ro("stats", &ir::ResultIR::stats);

    // ── Preprocessed handle ─────────────────────────────────────────────
    nb::class_<Preprocessed>(m, "Preprocessed",
                             "Opaque handle owning a preprocessed problem. "
                             "Single-solve: artifacts are single-consumer.")
        .def_prop_ro("proven_unsolvable",
                     [](const Preprocessed& p) { return p.pre.proven_unsolvable; })
        .def_prop_ro("unsolvable_reason",
                     [](const Preprocessed& p) { return p.pre.unsolvable_reason; })
        .def_prop_ro("lower_bound",
                     [](const Preprocessed& p) { return p.pre.lower_bound; })
        .def_prop_ro("consumed",
                     [](const Preprocessed& p) { return p.consumed; })
        .def("interrupt",
             [](Preprocessed& p) { p.interrupt_slot.interrupt(); },
             "Cancel the solve currently running on this handle, from any "
             "thread (flips Z3's interrupt flag; near-instant, no GIL "
             "gymnastics). The interrupted solve returns promptly with "
             "TIMEOUT status. Only the Z3 search is interruptible: during "
             "preprocessing or other non-Z3 phases this is a no-op.");

    // ── Functions ───────────────────────────────────────────────────────
    m.def("preprocess", &preprocess_ir, nb::arg("problem"), nb::arg("args"),
          "Run the strategy-independent preprocessing pipeline on a ProblemIR "
          "with the main CLI's flags; returns a Preprocessed handle. The handle "
          "remembers the args (solve()/solve_anytime() reuse them by default). "
          "The GIL is released while preprocessing.");

    m.def("solve", &solve_handle, nb::arg("preprocessed"), nb::arg("args") = nb::none(),
          "Resolve the configured strategy and search; returns a ResultIR. "
          "args=None (the default) reuses the handle's preprocess() args. "
          "Passing a list replaces them for this solve -- expert-only: "
          "option-dependent preprocessing decisions (mode, symmetries, RPG "
          "bounds) are already baked into the handle and are NOT recomputed. "
          "Consumes the handle (single-solve). The GIL is released while "
          "solving.");

    m.def("solve_anytime", &solve_anytime_handle,
          nb::arg("preprocessed"), nb::arg("on_plan"), nb::arg("args") = nb::none(),
          "Like solve(), but additionally invokes on_plan(list[ActionInstanceIR]) "
          "with each plan the search reports (every improving plan in anytime/"
          "optimal mode). The callback runs on the solver thread with the GIL "
          "re-acquired: keep it cheap and do not re-enter this module from it. "
          "args has the same reuse-by-default semantics as solve().");

    m.def("set_log_sink", &set_log_sink, nb::arg("sink").none(),
          "Redirect the planner's log output. sink(level: int, text: str) "
          "receives each formatted line (no trailing newline, already "
          "verbosity-filtered; level: 0=error, 1=info, 2=verbose, 3=debug); "
          "None restores the default (stdout; errors to stderr). The sink is "
          "process-global and NOT synchronized against a running solve: set "
          "and clear it from the thread that calls preprocess()/solve(). It "
          "runs on the solver thread with the GIL re-acquired -- keep it "
          "cheap and do not re-enter this module from it.");

    m.def("export_formula", &export_formula_ir, nb::arg("problem"), nb::arg("args"),
          "Export the SMT formula for a fixed timestep instead of solving "
          "(args must include --formula-timestep; strategy must support it).");

    m.def("list_strategies", &StrategyRegistry::list_strategies,
          "Names of all registered strategies.");

    m.def("z3_version", &z3_version,
          "Version of the libz3 this module is linked against (must match "
          "the z3-solver package's, or two copies are loaded).");
}
