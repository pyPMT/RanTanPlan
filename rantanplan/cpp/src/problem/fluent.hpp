#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <optional>
#include "parameter.hpp"
#include "type.hpp"
#include "expr_pool.hpp"  // ExprID (declared_default_ references a pool constant)
#include "rational.hpp"

namespace rantanplan {

/**
 * @brief Fluent
 * 
 * Represents a state-dependent variable (fluent) in the planning domain.
 */
class Fluent {
public:
    // Constructors
    Fluent() : id_(-1) {}
    Fluent(const std::string& name, const Type* value_type)
        : name_(name), value_type_(value_type), id_(-1) {}
    Fluent(const std::string& name, const Type* value_type,
           const std::vector<Parameter>& parameters)
        : name_(name), value_type_(value_type), parameters_(parameters), id_(-1) {}
    // Accessors
    const std::string& name() const { return name_; }
    int id() const { return id_; }
    const Type* value_type() const { return value_type_; }
    const std::vector<Parameter>& parameters() const { return parameters_; }
    size_t parameter_count() const { return parameters_.size(); }
    const Parameter& parameter(size_t index) const { return parameters_[index]; }
    
    // Setters
    void set_name(const std::string& name) { name_ = name; }
    void set_id(int id) { id_ = id; }
    void set_value_type(const Type* value_type) { value_type_ = value_type; }
    void add_parameter(const Parameter& param) { parameters_.push_back(param); }
    void set_parameters(const std::vector<Parameter>& parameters) { parameters_ = parameters; }

    // Declared numeric bounds (from bounded int/real value types, e.g.
    // integer[0, 15]), exact. Either side may be unset for a one-sided bound.
    void set_declared_bounds(std::optional<Rational> lower, std::optional<Rational> upper) {
        lower_bound_ = lower;
        upper_bound_ = upper;
    }
    std::optional<Rational> declared_lower_bound() const { return lower_bound_; }
    std::optional<Rational> declared_upper_bound() const { return upper_bound_; }
    bool has_declared_bounds() const {
        return lower_bound_.has_value() || upper_bound_.has_value();
    }

    // Declared default_initial_value (UP `fluents_defaults`), as a CONSTANT
    // ExprID in the problem's pool. Authoritative for cells/fluents missing an
    // explicit initial value; the CWA pass falls back to clamped-0/false when
    // it is unset. Array cells inherit their array fluent's default.
    void set_declared_default(ExprID value) { declared_default_ = value; }
    std::optional<ExprID> declared_default() const { return declared_default_; }

    // Array fluents carry a first-class array value type (see Type::is_array);
    // the per-dimension sizes and the leaf element type are derived from it.
    // Undefined positions (full coordinate tuples) are per-fluent data, not
    // part of the type: two fluents of one array type may exclude different
    // cells. The lowering mints one scalar cell per defined position, of the
    // leaf element type with the declared (element) bounds above.
    bool is_array() const { return value_type_ && value_type_->is_array(); }
    std::vector<int64_t> array_sizes() const {
        std::vector<int64_t> sizes;
        for (const Type* t = value_type_; t && t->is_array(); t = t->array_element()) {
            sizes.push_back(t->array_size());
        }
        return sizes;
    }
    const Type* array_leaf_element_type() const {
        const Type* t = value_type_;
        while (t && t->is_array()) t = t->array_element();
        return t;
    }
    void set_array_undefined_positions(
            std::vector<std::vector<int64_t>> undefined_positions) {
        array_undefined_positions_ = std::move(undefined_positions);
    }
    const std::vector<std::vector<int64_t>>& array_undefined_positions() const {
        return array_undefined_positions_;
    }

    // Set fluents carry a first-class set value type (see Type::is_set_type).
    // A set fluent is lowered into a boolean membership fluent (an element
    // parameter prepended to the signature) by SetLoweringPass before grounding;
    // no downstream consumer sees a set-typed fluent. Nested set types are
    // rejected at conversion, so the element type is always scalar/object.
    bool is_set_fluent() const { return value_type_ && value_type_->is_set_type(); }
    const Type* set_element_type() const {
        return value_type_ ? value_type_->set_element() : nullptr;
    }

    // Convenience methods
    bool is_predicate() const {
        return value_type_ && value_type_->is_bool();
    }
    bool is_function() const {
        return value_type_ && !value_type_->is_bool();
    }
    
    // Type checking methods using the Type class
    bool is_bool_fluent() const {
        return value_type_ && value_type_->is_bool();
    }
    bool is_int_fluent() const {
        return value_type_ && value_type_->is_int();
    }
    bool is_real_fluent() const {
        return value_type_ && value_type_->is_real();
    }
    bool is_object_fluent() const {
        return value_type_ && value_type_->is_object();
    }
    
    // String representation
    std::string to_string() const;
    
    // Operators
    bool operator==(const Fluent& other) const;
    bool operator!=(const Fluent& other) const { return !(*this == other); }

private:
    std::string name_;
    int id_;
    const Type* value_type_ = nullptr;
    std::vector<Parameter> parameters_;
    std::optional<Rational> lower_bound_;
    std::optional<Rational> upper_bound_;
    std::optional<ExprID> declared_default_;
    std::vector<std::vector<int64_t>> array_undefined_positions_;
};

} // namespace rantanplan

// specialising std::hash for Fluent by using its ID
// note that for this to work we need to be in the std namespace
namespace std {
    template<>
    struct hash<rantanplan::Fluent> {
        std::size_t operator()(const rantanplan::Fluent& fluent) const {
            return std::hash<int>()(fluent.id());
        }
    };
}
