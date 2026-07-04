#include "stats.hpp"
#include <iostream>
#include <iomanip>

namespace rantanplan {

std::unique_ptr<Stats> Stats::instance_;

Stats& Stats::instance() {
    if (!instance_) {
        instance_ = std::unique_ptr<Stats>(new Stats());
    }
    return *instance_;
}

void Stats::set(const std::string& key, double value) {
    std::lock_guard<std::mutex> lock(mutex_);
    stats_[key] = value;
}

double Stats::get(const std::string& key) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = stats_.find(key);
    return (it != stats_.end()) ? it->second : 0.0;
}

void Stats::add(const std::string& key, double value) {
    std::lock_guard<std::mutex> lock(mutex_);
    stats_[key] += value;  // If key doesn't exist, it starts at 0
}

void Stats::set_multiple(const std::map<std::string, double>& values) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto& pair : values) {
        stats_[pair.first] = pair.second;
    }
}

void Stats::set_prefixed(const std::string& prefix, const std::string& key, double value) {
    std::lock_guard<std::mutex> lock(mutex_);
    stats_[prefix + "." + key] = value;
}

void Stats::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    stats_.clear();
}

void Stats::print_all() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stats_.empty()) {
        std::cout << "No statistics collected." << std::endl;
        return;
    }

    std::cout << "\n=== Statistics ===" << std::endl;
    for (const auto& pair : stats_) {
        std::cout << pair.first << ": " << std::fixed << std::setprecision(3) << pair.second << std::endl;
    }
    std::cout << "==================" << std::endl;
}

const std::map<std::string, double>& Stats::all() const {
    return stats_;
}

size_t Stats::size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return stats_.size();
}

} // namespace rantanplan