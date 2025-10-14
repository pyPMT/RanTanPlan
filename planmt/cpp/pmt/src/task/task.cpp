


#include <fstream>
#include <spdlog/spdlog.h>

#include "utils/templates.hpp"
#include "task.hpp"

namespace planmt {

Task::Task(const std::string& pb_problemfile):z3_ctx_(z3::context()) {
    // Load the protobuf file
    spdlog::info("Reading the planning task from file: {}", pb_problemfile);
    std::fstream input(pb_problemfile, std::ios::in | std::ios::binary);
    pb_problem_ = pb::Problem();
    if (!input || !pb_problem_.ParseFromIstream(&input)) {
        throw std::runtime_error("Could not open or parse problem file: " + pb_problemfile);
    }
    
    spdlog::info("Loading the task...");
    load_task();
    
    spdlog::info("Successfully loaded problem from: {}.", pb_problemfile);
}


void Task::load_task(void) {
    // First we need to load the types.
    load_types(pb_problem_.types());

    // Then we can load the objects.
    load_objects(pb_problem_.objects());

}

void Task::load_types(const pb::RepeatedTypeDeclaration& pb_types) {
    types_.clear();
    type_name_to_ptr_.clear();

    // ensure basic types are present
    types_.emplace_back("up:bool");
    types_.emplace_back("up:int");
    types_.emplace_back("up:real");

    for (const auto& pb_type : pb_types) {
        types_.emplace_back(pb_type.type_name());
        types_.back().set_parent_name(pb_type.parent_type());
    }
    
    // Build the name to pointer mapping
    for (auto& type : types_) {
        type_name_to_ptr_[type.name()] = type;
    }

    // Resolve parent pointers
    for (auto& type : types_) {
        if (!type.parent_name().empty()) {
            
            type.set_parent(&type_name_to_ptr_[type.parent_name()]);
            
            // auto it = type_name_to_ptr_.find(type.parent_name());
            // if (it != type_name_to_ptr_.end()) {
            //     type.set_parent(it->second);
            // }
        }
    }
    
}

void Task::load_objects(const pb::RepeatedObjectDeclaration& pb_objects) {
    objects_.clear();
    for (const auto& pb_obj : pb_objects) {
        auto &type = type_name_to_ptr_[pb_obj.type()]; // This should crash if type not found
        // auto &type = find_type(pb_obj.type());
        // objects_.emplace_back(pb_obj.name(), type);
    }
}



} // namespace planmt