#include "type.hpp"

namespace rantanplan {

namespace {
/// The primitive/object kind implied by a type name. Structured kinds
/// (array/set) are not name-derived — they are stamped by set_array/set_set.
TypeKind primitive_kind(const std::string& name) {
    if (name == "bool" || name == "up:bool") return TypeKind::BOOL;
    if (name == "int" || name == "up:int" ||
        name == "integer" || name == "up:integer") return TypeKind::INT;
    if (name == "real" || name == "up:real") return TypeKind::REAL;
    return TypeKind::OBJECT;
}
}  // namespace

Type::Type(const std::string& name) : name_(name), kind_(primitive_kind(name)) {}

const std::string& Type::name() const { return name_; }
void Type::set_name(const std::string& name) {
    name_ = name;
    // Re-derive only the primitive/object kind; an array/set kind is established
    // by set_array/set_set after construction and must not be clobbered.
    if (kind_ != TypeKind::ARRAY && kind_ != TypeKind::SET)
        kind_ = primitive_kind(name);
}

const Type* Type::parent() const { return parent_; }
void Type::set_parent(const Type* parent) { parent_ = parent; }

void Type::set_parent_name(const std::string& parent_name) { parent_name_ = parent_name; }
const std::string& Type::parent_name() const { return parent_name_; }

bool Type::is_subtype_of(const Type* supertype) const {
    const Type* current = this;
    while (current) {
        if (current == supertype) return true;
        current = current->parent();
    }
    return false;
}

} // namespace rantanplan
