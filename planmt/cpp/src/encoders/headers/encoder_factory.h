#pragma once

#include <memory>
#include <string>
#include <z3++.h>

#include "base_encoder.h"
#include "grounded_encoder.h"
#include "chained_grounded_encoder.h"
#include "r2e_grounded_encoder.h"
#include "reified_grounded_encoder.h"
#include "problem/headers/problem.h"

namespace planmt {

class EncoderFactory {
public:
    enum class EncoderType {
        GROUNDED,
        CHAINED,
        R2E,
        REIFIED
    };
    
    static std::unique_ptr<BaseEncoder> create_encoder(EncoderType type, const Problem& problem, z3::context& ctx);
    static std::unique_ptr<BaseEncoder> create_encoder(const std::string& type_name, const Problem& problem, z3::context& ctx);
    
private:
    static EncoderType string_to_type(const std::string& type_name);
};

} // namespace planmt
