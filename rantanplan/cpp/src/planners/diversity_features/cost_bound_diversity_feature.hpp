#pragma once

#include <iostream>

#include "base_diversity_feature.hpp"

namespace rantanplan {

    class CostBoundDiversityFeature : public BaseDiversityFeature {
    public:
        CostBoundDiversityFeature(std::string args, BaseEncoder& encoder) : BaseDiversityFeature("CostBoundDiversityFeature", args, encoder) {
            
            // We need to get the actions per step and then sum the number of actions selected.
            
            
            std::cout << "Initializing CostBoundDiversityFeature with args: " << args << std::endl;
        }
    };


}