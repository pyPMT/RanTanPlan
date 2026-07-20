#pragma once

#include <cstdint>
#include <cstddef>
#include <functional>
#include <vector>

/**
 * @file state_valuation.hpp
 * @brief Plain-data valuation of every ground fluent (Z3-free).
 *
 * Indexed by grounded fluent id (position in Problem::grounded_fluents()).
 * Consumed by the NumericRelaxedPlanningGraph initial-state override, the
 * anchor-search planner's candidate scoring, blocking, and SMT init
 * override, and the replay check.  Extraction contract (see extract_state
 * in AnchorSearchPlanner):
 * dynamic fluents are read from the Z3 model with model_completion, static
 * fluents (never targeted by any effect) are filled from the problem's
 * initial state — only dynamic fluents have Z3 state variables.
 */
namespace rantanplan {

struct StateValuation {
    enum class Kind : uint8_t { Unset, Boolean, Numeric, Object };

    struct FluentValue {
        Kind kind = Kind::Unset;
        bool bool_value = false;
        double numeric_value = 0.0;
        int object_index = -1;

        static FluentValue make_bool(bool v) {
            FluentValue fv; fv.kind = Kind::Boolean; fv.bool_value = v; return fv;
        }
        static FluentValue make_numeric(double v) {
            FluentValue fv; fv.kind = Kind::Numeric; fv.numeric_value = v; return fv;
        }
        static FluentValue make_object(int idx) {
            FluentValue fv; fv.kind = Kind::Object; fv.object_index = idx; return fv;
        }

        bool operator==(const FluentValue& other) const {
            if (kind != other.kind) return false;
            switch (kind) {
                case Kind::Unset:   return true;
                case Kind::Boolean: return bool_value == other.bool_value;
                case Kind::Numeric: return numeric_value == other.numeric_value;
                case Kind::Object:  return object_index == other.object_index;
            }
            return false;
        }
        bool operator!=(const FluentValue& other) const { return !(*this == other); }
    };

    std::vector<FluentValue> values;  // indexed by grounded fluent id

    bool operator==(const StateValuation& other) const {
        return values == other.values;
    }
    bool operator!=(const StateValuation& other) const { return !(*this == other); }

    size_t hash() const {
        size_t h = values.size();
        for (const auto& v : values) {
            size_t x = static_cast<size_t>(v.kind);
            switch (v.kind) {
                case Kind::Boolean: x ^= v.bool_value ? 0x9e3779b9u : 0x85ebca6bu; break;
                case Kind::Numeric: x ^= std::hash<double>()(v.numeric_value);    break;
                case Kind::Object:  x ^= std::hash<int>()(v.object_index);        break;
                case Kind::Unset:   break;
            }
            h ^= x + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2);
        }
        return h;
    }
};

} // namespace rantanplan
