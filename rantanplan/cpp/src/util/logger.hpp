#pragma once

#include <functional>
#include <string>
#include <vector>
#include <map>
#include <utility>
#include <memory>
#include "verbosity.hpp"

namespace rantanplan {

/**
 * @brief Centralized logging with consistent formatting and verbosity control
 *
 * Replaces scattered std::cout statements with structured, verbosity-aware output.
 * All output goes through this class, ensuring consistent formatting and respecting
 * the global verbosity settings.
 *
 * Usage:
 *   Logger::instance().info("Starting RPG construction");
 *   Logger::instance().component(VerbosityLevel::INFO, "RPG.Boolean", {
 *       {"time", "147ms"},
 *       {"layers", "15"},
 *       {"mem", "12MB"}
 *   });
 */
class Logger {
public:
    // Receives each formatted output line (no trailing newline) after
    // verbosity filtering. The level is the line's minimum verbosity;
    // errors arrive as SILENT (they always pass the filter).
    using Sink = std::function<void(VerbosityLevel, const std::string&)>;

    // Get the global logger instance
    static Logger& instance();

    // Set the verbosity threshold (the driver snapshots it from Config once
    // at startup; util/ itself has no Config dependency)
    void set_verbosity(VerbosityLevel verbosity);

    // Redirect output. An empty sink restores the default (stdout; errors to
    // stderr). The Logger is process-global: installers must reset the sink
    // before their scope ends (see ScopedLogSink in bindings.cpp).
    void set_sink(Sink sink);

    // Simple message logging (respects verbosity)
    void info(const std::string& msg);
    void verbose(const std::string& msg);
    void debug(const std::string& msg);
    void error(const std::string& msg);  // Always shown

    // Structured component output with consistent formatting
    // Format: [ComponentName]  time: Xms  |  key1: value1  |  key2: value2  |  mem: XMB
    void component(
        VerbosityLevel min_level,
        const std::string& component_name,
        const std::vector<std::pair<std::string, std::string>>& metrics
    );

    // Timestep-specific output for search loop
    // Format: [Encoding T0]  time: Xms  |  constraints: N  |  mem: XMB
    // Format: [Solving T0]   time: Xms  |  result: SAT/UNSAT
    void timestep_encoding(
        VerbosityLevel min_level,
        int timestep,
        const std::vector<std::pair<std::string, std::string>>& metrics
    );

    void timestep_solving(
        VerbosityLevel min_level,
        int timestep,
        const std::vector<std::pair<std::string, std::string>>& metrics,
        const std::string& prefix = "T"
    );

    // Summary box for final results
    // Format:
    // === Planning Summary ===
    // Total time:     1.234s
    // Plan found:     YES
    // ...
    // ========================
    void summary(const std::map<std::string, std::string>& data);

    // Enable/disable all output (for testing)
    void set_enabled(bool enabled);

    // Check if we should print at this level (public so callers can guard
    // expensive message construction without reading Config themselves)
    bool should_print(VerbosityLevel min_level) const;

private:
    Logger() = default;

    // Single exit point: every method writes its already-formatted,
    // verbosity-filtered line through here (default: stdout, errors to
    // stderr; or the installed sink)
    void emit(VerbosityLevel level, const std::string& text);

    // Formatting helpers
    std::string format_component_line(
        const std::string& component_name,
        const std::vector<std::pair<std::string, std::string>>& metrics
    ) const;

    VerbosityLevel verbosity_ = VerbosityLevel::INFO;
    bool enabled_ = true;
    Sink sink_;

    static constexpr int COMPONENT_WIDTH = 16;  // Width for component name column

    static std::unique_ptr<Logger> instance_;
};

} // namespace rantanplan
