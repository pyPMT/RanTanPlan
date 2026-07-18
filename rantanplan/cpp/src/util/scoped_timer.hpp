#pragma once

#include <string>
#include <chrono>

namespace rantanplan {

/**
 * @brief RAII timer that automatically records elapsed time to Stats singleton
 *
 * Usage:
 *   {
 *       ScopedTimer timer("rpg.build_time_ms");
 *       // ... do work ...
 *   }  // Automatically records timing to Stats on destruction
 *
 * You can also query elapsed time before destruction:
 *   ScopedTimer timer("key");
 *   // ... do some work ...
 *   double elapsed = timer.elapsed_ms();
 */
class ScopedTimer {
public:
    /**
     * @brief Construct timer and start measuring
     * @param stats_key Key to use when recording to Stats singleton
     */
    explicit ScopedTimer(const std::string& stats_key);

    /**
     * @brief Construct a measuring-only timer that records nothing to Stats.
     *
     * For log lines that want an elapsed time without duplicating a stats
     * key someone else already records (e.g. the pipeline's per-pass timer).
     */
    ScopedTimer();

    /**
     * @brief Destructor - records elapsed time to Stats (keyed timers only)
     */
    ~ScopedTimer();

    /**
     * @brief Get elapsed time in milliseconds (without destroying timer)
     */
    double elapsed_ms() const;

    /**
     * @brief Get elapsed time in seconds (without destroying timer)
     */
    double elapsed_s() const;

    // Disable copy and move
    ScopedTimer(const ScopedTimer&) = delete;
    ScopedTimer& operator=(const ScopedTimer&) = delete;
    ScopedTimer(ScopedTimer&&) = delete;
    ScopedTimer& operator=(ScopedTimer&&) = delete;

private:
    std::string stats_key_;
    std::chrono::high_resolution_clock::time_point start_;
};

} // namespace rantanplan
