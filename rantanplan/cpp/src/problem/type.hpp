#pragma once

#include <cstdint>
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

    // Primitive type helpers
    bool is_bool() const { return name_ == "bool" || name_ == "up:bool"; }
    bool is_int() const { return name_ == "int" || name_ == "up:int" || name_ == "integer" || name_ == "up:integer"; }
    bool is_real() const { return name_ == "real" || name_ == "up:real"; }
    bool is_object() const { return !is_bool() && !is_int() && !is_real() && !is_array(); }

    // Numeric bounds are deliberately NOT part of the type. A bounded integer
    // (UP's IntType(lb, ub), e.g. integer[0, 15]) is the base `int` type here,
    // with the bounds carried as variable-level data (Fluent::declared_bounds,
    // range-variable domains). Considered and rejected: promoting bounded ints
    // to a first-class Type to mirror UP — it would multiply the type table
    // (one entry per distinct [lb, ub]) and ripple through every is_int() check
    // and the numeric machinery that already threads bounds as data, for no
    // real gain. Arrays *are* a type (below) because an array value has
    // structure the grounder must destructure into cells; a bounded int is a
    // scalar carrying metadata, which belongs on the variable.

    // Array types (name "array[N, <element>]", nested per dimension) are
    // first-class entries in the problem's type table, registered at IR load
    // from each array fluent's element type + per-dimension sizes. The element
    // of a multi-dimensional array is itself a registered array type; the leaf
    // element is always a scalar type.
    //
    // Examples (each indented line is one entry in the type table):
    //   row : int[4]
    //       int                        (scalar leaf)
    //       array[4, int]              element -> int,           size 4
    //   grid : int[3][4]
    //       int                        (scalar leaf)
    //       array[4, int]              element -> int,           size 4
    //       array[3, array[4, int]]    element -> array[4, int], size 3
    // A Fluent of the outer type reports array_sizes() == [4] resp. [3, 4], and
    // array_leaf_element_type() == int in both cases (see Fluent::array_sizes).
    //
    // Registration (ir_io.cpp): nested names are built innermost-first; entries
    // are emplaced into the type table *before* the name->pointer map is built,
    // so vector reallocation never dangles a Type*, and element pointers are
    // linked in a second pass once every entry has a stable address.
    bool is_array() const { return array_element_ != nullptr; }
    const Type* array_element() const { return array_element_; }
    int64_t array_size() const { return array_size_; }
    void set_array(const Type* element, int64_t size) {
        array_element_ = element;
        array_size_ = size;
    }

private:
    std::string name_;
    const Type* parent_ = nullptr;
    std::string parent_name_;
    const Type* array_element_ = nullptr;
    int64_t array_size_ = 0;
};

} // namespace rantanplan
