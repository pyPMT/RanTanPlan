#pragma once

#include <string>
#include <memory>

namespace planmt {

class Type {
public:
    Type(): name_("object"), parent_(nullptr) {};
    Type(const std::string& name) : name_(name) {};

    const std::string& name() const { return name_; }
    void set_name(const std::string& name) { name_ = name; }

    // Parent type for hierarchy (nullptr if root)
    const Type* parent() const { return parent_; }
    void set_parent(const Type* parent) { parent_ = parent; }

    // For initial parsing, store parent name until resolved
    void set_parent_name(const std::string& parent_name) { parent_name_ = parent_name; }
    const std::string& parent_name() const { return parent_name_; }

    // Subtype check (traverse up the hierarchy)
    bool is_subtype_of(const Type* supertype) const {
        const Type* current = this;
        while (current) {
            if (current == supertype) return true;
            current = current->parent();
        }
        return false;
    }

    // Primitive type helpers
    bool is_bool() const   { return name_ == "bool" || name_ == "up:bool"; }
    bool is_int() const    { return name_ == "int" || name_ == "up:int" || name_ == "integer" || name_ == "up:integer"; }
    bool is_real() const   { return name_ == "real" || name_ == "up:real"; }
    bool is_object() const { return !is_bool() && !is_int() && !is_real(); }

private:
    std::string name_;
    const Type* parent_ = nullptr;
    std::string parent_name_;
};

} // namespace planmt
