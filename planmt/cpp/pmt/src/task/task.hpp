#pragma once

#include <memory>
#include <string>
#include <vector>
#include <z3++.h>
#include <variant>

#include "protobuf_aliases.hpp"
#include "elements/type.hpp"
#include "elements/object.hpp"

namespace planmt {

using up_types = std::variant<Type>;


class Task {
public:
    Task(const std::string& pb_problemfile);
    ~Task() = default;

private:
    // A protobuf problem representation.
    pb::Problem pb_problem_;

    // A z3 context for all z3 objects.
    z3::context z3_ctx_;

    // UP Problem elements
    std::vector<Type>   types_;
    std::vector<Object> objects_;

    // UP Lookup map.
    std::unordered_map<std::string, Type> type_name_to_ptr_;
    std::unordered_map<std::string, size_t> object_name_to_index_;

    // Load the task from the protobuf representation.
    void load_task(void);

    // Read problem types
    void load_types(const pb::RepeatedTypeDeclaration& pb_types);

    // Search for types.
    // const Type& find_type(const std::string& name) const;
    
    // Read problem objects
    void load_objects(const pb::RepeatedObjectDeclaration& pb_objects);
};
}