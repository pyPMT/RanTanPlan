


#include <fstream>
#include <spdlog/spdlog.h>

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
    spdlog::info("Successfully loaded problem from: {}", pb_problemfile);

    std::cout << std::endl;
    
}

} // namespace planmt