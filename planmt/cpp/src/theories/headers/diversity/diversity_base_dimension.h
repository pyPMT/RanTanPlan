#pragma once

#include <string>
#include <vector>

#include <z3++.h>

#include "problem/headers/problem.h"
#include "encoders/headers/base_encoder.h"

namespace planmt {

using DiversityValue = std::variant<int, double, std::string, z3::expr>;

struct dimension_value_t {
    std::string name;
    DiversityValue value;
    dimension_value_t(std::string a, DiversityValue b)
        : name(std::move(a)), value(std::move(b)) {}
};

using encoder_fn_t = std::function<std::vector<z3::expr>(size_t)>;
using extract_fn_t = std::function<dimension_value_t(const z3::model&)>;

class DiversityDimension {
public:
    virtual ~DiversityDimension() = default;

    // Make these virtual (prefer pure virtual so you can't forget to implement)
    virtual std::vector<z3::expr> encode(size_t timestep) = 0;
    virtual std::unique_ptr<dimension_value_t> extract(const z3::model& model) = 0;

protected:
    DiversityDimension(const Problem& problem, BaseEncoder& encoder, std::string name);
    const std::string name_;
    const Problem& problem_;
    BaseEncoder& encoder_;
    z3::context& ctx_;
    std::unordered_map<std::string, extract_fn_t> extractors;
    std::unordered_map<std::string, encoder_fn_t> encoders;
};


}
