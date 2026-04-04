#pragma once

#include <string>
#include "../../encoders/base_encoder.hpp"

namespace rantanplan {

// ---------------------------------------------------------------------------
// BaseDiversityFeature
// ---------------------------------------------------------------------------

/**
 * @brief Base interface for diversity features.
 *
 */

class BaseDiversityFeature {
public:
    BaseDiversityFeature(std::string name, std::string args, BaseEncoder& encoder) {}
};



}