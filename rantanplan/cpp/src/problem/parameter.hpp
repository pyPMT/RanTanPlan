#pragma once

#include <string>
#include <cstdint>
#include <optional>
#include "type.hpp"

namespace rantanplan {

/**
 * @brief Parameter
 *
 * Represents a typed parameter for actions or fluents.
 */
class Parameter {
public:
    // Constructors
    Parameter() = default;
    Parameter(const std::string& name, const Type* type) : name_(name), type_(type) {}

    // Accessors
    const std::string& name() const { return name_; }
    const Type* type() const { return type_; }

    /// A finite-domain numeric (bounded-int) parameter: the grounder enumerates
    /// its [lower_bound, upper_bound] interval instead of type-compatible objects.
    bool is_numeric() const { return type_ && type_->is_int(); }
    std::optional<int64_t> lower_bound() const { return lower_bound_; }
    std::optional<int64_t> upper_bound() const { return upper_bound_; }

    // Setters
    void set_name(const std::string& name) { name_ = name; }
    void set_type(const Type* type) { type_ = type; }
    void set_lower_bound(int64_t v) { lower_bound_ = v; }
    void set_upper_bound(int64_t v) { upper_bound_ = v; }

    // String representation
    std::string to_string() const { return name_ + " : " + (type_ ? type_->name() : ""); }
    
    // Operators
    bool operator==(const Parameter& other) const {
        return name_ == other.name_ && type_ == other.type_;
    }
    bool operator!=(const Parameter& other) const { return !(*this == other); }

private:
    std::string name_;
    const Type* type_ = nullptr;
    std::optional<int64_t> lower_bound_;
    std::optional<int64_t> upper_bound_;
};

} // namespace rantanplan
