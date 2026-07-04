#include "logger.hpp"
#include <iostream>
#include <iomanip>
#include <sstream>

namespace rantanplan {

std::unique_ptr<Logger> Logger::instance_;

Logger& Logger::instance() {
    if (!instance_) {
        instance_ = std::unique_ptr<Logger>(new Logger());
    }
    return *instance_;
}

void Logger::set_verbosity(VerbosityLevel verbosity) {
    verbosity_ = verbosity;
}

void Logger::set_sink(Sink sink) {
    sink_ = std::move(sink);
}

void Logger::emit(VerbosityLevel level, const std::string& text) {
    if (sink_) {
        sink_(level, text);
    } else if (level == VerbosityLevel::SILENT) {
        // Only errors are emitted at SILENT (always-print) level
        std::cerr << text << std::endl;
    } else {
        std::cout << text << std::endl;
    }
}

bool Logger::should_print(VerbosityLevel min_level) const {
    if (!enabled_) {
        return false;
    }

    // Error messages always print
    if (min_level == VerbosityLevel::SILENT) {
        return true;
    }

    // Check against configured verbosity (defaults to INFO until set)
    return verbosity_ >= min_level;
}

void Logger::info(const std::string& msg) {
    if (should_print(VerbosityLevel::INFO)) {
        emit(VerbosityLevel::INFO, msg);
    }
}

void Logger::verbose(const std::string& msg) {
    if (should_print(VerbosityLevel::VERBOSE)) {
        emit(VerbosityLevel::VERBOSE, msg);
    }
}

void Logger::debug(const std::string& msg) {
    if (should_print(VerbosityLevel::DEBUG)) {
        emit(VerbosityLevel::DEBUG, msg);
    }
}

void Logger::error(const std::string& msg) {
    // Errors always print, bypassing the verbosity filter
    emit(VerbosityLevel::SILENT, "Error: " + msg);
}

std::string Logger::format_component_line(
    const std::string& component_name,
    const std::vector<std::pair<std::string, std::string>>& metrics
) const {
    std::ostringstream oss;

    // Component name in brackets, left-aligned in fixed width
    oss << "[" << std::left << std::setw(COMPONENT_WIDTH - 2) << component_name << "]";

    // Metrics separated by " | "
    bool first = true;
    for (const auto& metric : metrics) {
        if (!first) {
            oss << "  |  ";
        } else {
            oss << "  ";
            first = false;
        }
        oss << metric.first << ": " << metric.second;
    }

    return oss.str();
}

void Logger::component(
    VerbosityLevel min_level,
    const std::string& component_name,
    const std::vector<std::pair<std::string, std::string>>& metrics
) {
    if (should_print(min_level)) {
        emit(min_level, format_component_line(component_name, metrics));
    }
}

void Logger::timestep_encoding(
    VerbosityLevel min_level,
    int timestep,
    const std::vector<std::pair<std::string, std::string>>& metrics
) {
    if (should_print(min_level)) {
        std::ostringstream component_name;
        component_name << "Encoding T" << timestep;
        emit(min_level, format_component_line(component_name.str(), metrics));
    }
}

void Logger::timestep_solving(
    VerbosityLevel min_level,
    int timestep,
    const std::vector<std::pair<std::string, std::string>>& metrics,
    const std::string& prefix
) {
    if (should_print(min_level)) {
        std::ostringstream component_name;
        component_name << "Solving " << prefix << timestep;
        emit(min_level, format_component_line(component_name.str(), metrics));
    }
}

void Logger::summary(const std::map<std::string, std::string>& data) {
    if (!should_print(VerbosityLevel::INFO)) {
        return;
    }

    std::ostringstream oss;
    oss << "\n=== Planning Summary ===\n";

    // Find the longest key for alignment
    size_t max_key_length = 0;
    for (const auto& pair : data) {
        max_key_length = std::max(max_key_length, pair.first.length());
    }

    // Print each key-value pair, aligned
    for (const auto& pair : data) {
        oss << std::left << std::setw(max_key_length + 1) << (pair.first + ":")
            << " " << pair.second << "\n";
    }

    oss << "========================";
    emit(VerbosityLevel::INFO, oss.str());
}

void Logger::set_enabled(bool enabled) {
    enabled_ = enabled;
}

} // namespace rantanplan
