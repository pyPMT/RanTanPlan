#pragma once

#include "../util/verbosity.hpp"
#include <string>

/**
 * @file config.hpp
 * @brief The CLI-facing singleton: parsed by CLIParser, validated by
 *        Config::validate(). Read by the backend and driver only -- the
 *        preprocessor takes PreprocessorOptions instead (see
 *        PreprocessorOptions::from_config, the one Config -> options
 *        translation site).
 */
namespace rantanplan {

/// Centralized configuration singleton, sectioned to mirror the CLI's flag
/// groups.
class Config {
public:
    static Config& instance();

    struct Global {
        VerbosityLevel verbosity = VerbosityLevel::INFO;
        bool debug_mode = false;
        int timeout = 0;              ///< 0 = no timeout (run indefinitely)
        std::string log_level = "INFO";
        bool persist_clauses = true;   ///< Retain propagator lemmas across backtracking; --no-persist-clauses discards them
        double epsilon = 1e-6;        ///< Numerical tolerance for floating-point comparisons
        bool reachability_grounding = true;  ///< --up-grounding disables this
        bool numeric_grounding = true;       ///< --no-numeric-grounding disables this
        bool cnf_normalization = true;       ///< --no-cnf-normalization disables this
    } global;

    /// Numeric Relaxed Planning Graph settings.
    struct RPG {
        bool enabled = true;            ///< Run NumericRPG preprocessing (lower bound + action removal)
        bool action_removal = true;     ///< Prune unreachable actions using RPG results
        bool use_interval_checker = true; ///< Interval-based formula evaluator (--smt-rpg-checker reverts to Z3)
        /// Stop when goals become achievable (before fixpoint). Default false:
        /// early termination is unsound for action removal because later RPG layers
        /// may discover actions reachable only through over-approximated bounds.
        bool early_goal_termination = false;
        /// Semantic goal-relevance analysis: uses AchieversAnalysis to remove
        /// actions that cannot contribute to any goal condition (even transitively).
        /// Runs after RPG action removal. Default true.
        bool goal_relevance = true;
        /// Maximum iterations for goal-relevance fixpoint loop.
        int goal_relevance_max_iterations = 10;
        /// Directional widening threshold: after this many consecutive expansions
        /// of the same side (lower/upper) of a numeric fluent's bounds, snap to +-inf.
        int widening_threshold = 100;
    } rpg;
    
    struct Planner {
        std::string strategy = "e-lin";  ///< Strategy name (looked up in StrategyRegistry)
        std::string mode = "satisficing";  ///< Search mode: satisficing, optimal, anytime
        /// Base path for plan files in IPC format (<path>.1, .2, ...). Empty =
        /// write none: the library writes files only when asked, and the
        /// command line (rantanplan/cli.py) asks for plan.txt by default.
        std::string output_plan = "";
        int max_steps = 500;
        int start_timestep = 0;  ///< Starting timestep for search (can be set by RPG lower bound)
        std::string horizon_schedule = "linear";  ///< Schedule type: linear|arithmetic|geometric|doubling
    } planner;

    /// Exists-step search (the ExistsCycle propagator module).
    struct Exists {
        bool full_cycle_search = false;  ///< --full-cycle-search: search for a cycle from every active action, not only from the one just fixed
    } exists;

    struct Symmetry {
        bool enable_symmetries = false;  ///< Enable symmetry detection
    } symmetry;

    struct FormulaExport {
        bool export_formula = false;      ///< Enable formula export mode
        int timestep = -1;                ///< Timestep to export formula for
        std::string output_file = "";     ///< Output file for formula (required)
    } formula_export;

    struct Logging {
        std::string log_file = "";        ///< Empty = no logging; path = solver decision log
    } logging;

    /// Re-entrant: every call rebuilds all sections from their defaults
    /// before parsing, so a second in-process run does not inherit the
    /// previous run's flags (or post-parse mutations like start_timestep).
    /// Concurrent solves in one process remain unsupported (singleton
    /// Config, global z3::set_param).
    void initialize(int argc, char* argv[]);
    void validate() const;

    bool is_silent() const { return global.verbosity == VerbosityLevel::SILENT; }
    bool is_info() const { return global.verbosity >= VerbosityLevel::INFO; }
    bool is_verbose() const { return global.verbosity >= VerbosityLevel::VERBOSE; }
    bool is_debug() const { return global.verbosity >= VerbosityLevel::DEBUG; }
    bool has_log_file() const { return !logging.log_file.empty(); }

private:
    Config() = default;
    Config(const Config&) = delete;
    Config& operator=(const Config&) = delete;
};

} // namespace rantanplan