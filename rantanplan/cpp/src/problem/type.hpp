#pragma once

#include <string>
#include <memory>

namespace rantanplan {

class Type {
public:
    Type() = default;
    explicit Type(const std::string& name);

    const std::string& name() const;
    void set_name(const std::string& name);

    // Parent type for hierarchy (nullptr if root)
    const Type* parent() const;
    void set_parent(const Type* parent);

    // For initial parsing, store parent name until resolved
    void set_parent_name(const std::string& parent_name);
    const std::string& parent_name() const;

    // Subtype check (traverse up the hierarchy)
    bool is_subtype_of(const Type* supertype) const;

    // Primitive type helpers.
    // A derived-bool (PDDL derived predicate) is boolean-valued, so is_bool() is
    // widened to include it: all boolean-encoding logic (Bool Z3 vars,
    // is_predicate) then treats a derived atom as a Bool.  The derived-aware seams
    // (frame suppression, CWA gating, completion-vs-propagator routing,
    // achiever/pruning) instead check is_derived_bool().
    bool is_derived_bool() const { return name_ == "derived_bool" || name_ == "up:derived_bool"; }
    bool is_bool() const { return name_ == "bool" || name_ == "up:bool" || is_derived_bool(); }
    bool is_int() const { return name_ == "int" || name_ == "up:int" || name_ == "integer" || name_ == "up:integer"; }
    bool is_real() const { return name_ == "real" || name_ == "up:real"; }
    bool is_object() const { return !is_bool() && !is_int() && !is_real(); }

private:
    std::string name_;
    const Type* parent_ = nullptr;
    std::string parent_name_;
};

} // namespace rantanplan
