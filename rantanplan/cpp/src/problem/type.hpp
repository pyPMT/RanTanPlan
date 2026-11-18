#pragma once

#include <cstdint>
#include <string>
#include <memory>

namespace rantanplan {

/// The single positive discriminator for a Type's shape. Set once at
/// construction from the type name (BOOL/INT/REAL, else OBJECT) and overridden
/// to ARRAY/SET by set_array/set_set. Adding a new kind adds one enumerator and
/// its is_*() accessor — no existing predicate (notably is_object) needs to
/// change, unlike a definition-by-exclusion.
enum class TypeKind { OBJECT, BOOL, INT, REAL, ARRAY, SET };

/// The built-in types' indices in every problem's type table (an
/// ExprNode::type_id): the loader registers them first, in this order, under
/// UP's names up:bool, up:integer and up:real. So there is one int type, and
/// code that makes a constant needs no lookup.
inline constexpr int BOOL_TYPE_ID = 0;
inline constexpr int INT_TYPE_ID = 1;
inline constexpr int REAL_TYPE_ID = 2;

class Type {
public:
    Type() = default;
    explicit Type(const std::string& name);

    const std::string& name() const;

    // Parent type for hierarchy (nullptr if root)
    const Type* parent() const;
    void set_parent(const Type* parent);

    // For initial parsing, store parent name until resolved
    void set_parent_name(const std::string& parent_name);
    const std::string& parent_name() const;

    // Subtype check (traverse up the hierarchy)
    bool is_subtype_of(const Type* supertype) const;

    // Kind discriminator (all shape predicates read this one field).
    TypeKind kind() const { return kind_; }
    bool is_bool() const { return kind_ == TypeKind::BOOL; }
    bool is_int() const { return kind_ == TypeKind::INT; }
    bool is_real() const { return kind_ == TypeKind::REAL; }
    bool is_object() const { return kind_ == TypeKind::OBJECT; }

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
    bool is_array() const { return kind_ == TypeKind::ARRAY; }
    const Type* array_element() const { return array_element_; }
    int64_t array_size() const { return array_size_; }
    void set_array(const Type* element, int64_t size) {
        array_element_ = element;
        array_size_ = size;
        kind_ = TypeKind::ARRAY;
    }

    // Set types (name "set{<element>}") are first-class type-table entries,
    // registered at IR load from each set fluent's element type. Unlike an
    // array a set has no fixed size (the element domain bounds it), so a set
    // type carries only its element. Set fluents are lowered into boolean
    // membership cells by SetLoweringPass before grounding; no later consumer
    // sees a set type. Nested set types are rejected at conversion, so
    // set_element_ is always a scalar/object type.
    bool is_set_type() const { return kind_ == TypeKind::SET; }
    const Type* set_element() const { return set_element_; }
    void set_set(const Type* element) { set_element_ = element; kind_ = TypeKind::SET; }

private:
    std::string name_;
    const Type* parent_ = nullptr;
    std::string parent_name_;
    const Type* array_element_ = nullptr;
    int64_t array_size_ = 0;
    const Type* set_element_ = nullptr;
    TypeKind kind_ = TypeKind::OBJECT;
};

} // namespace rantanplan
