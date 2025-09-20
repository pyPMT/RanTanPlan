#pragma once

#include <string>
#include <z3++.h>

#include "protobuf_aliases.hpp"

namespace planmt {
class Task {
public:
    Task(const std::string& pb_problemfile);
    ~Task() = default;

private:
    // A protobuf problem representation.
    pb::Problem pb_problem_;

    // A z3 context for all z3 objects.
    z3::context z3_ctx_;

};
}