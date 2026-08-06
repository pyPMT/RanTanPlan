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

/**
 * @brief RAII timer that adds its lifetime to a caller-owned counter.
 *
 * For code that is entered too often for ScopedTimer: it never touches the
 * Stats singleton (whose mutex would dominate a hot path), leaving the owner
 * to publish the accumulated total once, typically in its cleanup(). The Z3
 * propagator callbacks are the motivating case — fixed() runs hundreds of
 * millions of times on a large instance.
 *
 * Usage:
 *   void Foo::callback() {
 *       AccumulatingTimer timer(callback_ms_);   // member of Foo
 *       // ... do work ...
 *   }
 *
 * Pass enabled = false to skip both clock reads, for configurations where
 * there is nothing to attribute and the reads would only measure themselves.
 * Deliberately defined inline: an out-of-line constructor/destructor would
 * cost a call per entry on exactly the paths this exists for.
 *
 * Uses steady_clock rather than ScopedTimer's high_resolution_clock, which is
 * not guaranteed monotonic and so is not sound for accumulating durations.
 */
class AccumulatingTimer {
public:
    explicit AccumulatingTimer(double& accumulator_ms, bool enabled = true)
        : accumulator_ms_(accumulator_ms)
        , enabled_(enabled)
        , start_(enabled ? std::chrono::steady_clock::now()
                         : std::chrono::steady_clock::time_point{}) {
    }

    ~AccumulatingTimer() {
        if (!enabled_) return;
        auto end = std::chrono::steady_clock::now();
        accumulator_ms_ +=
            std::chrono::duration<double, std::milli>(end - start_).count();
    }

    // Disable copy and move
    AccumulatingTimer(const AccumulatingTimer&) = delete;
    AccumulatingTimer& operator=(const AccumulatingTimer&) = delete;
    AccumulatingTimer(AccumulatingTimer&&) = delete;
    AccumulatingTimer& operator=(AccumulatingTimer&&) = delete;

private:
    double& accumulator_ms_;
    bool enabled_;
    std::chrono::steady_clock::time_point start_;
};

} // namespace rantanplan
