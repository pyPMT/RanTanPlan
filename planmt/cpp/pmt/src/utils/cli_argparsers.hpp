#pragma once

#include <argparse/argparse.hpp>

namespace planmt {
    namespace utils {
        // Define string constants to use for the flags names.
        const std::string FLAG_INPUT_SHORT  = "-p";
        const std::string FLAG_OUTPUT_SHORT = "-o";
        const std::string FLAG_SEARCH_SHORT = "-s";
        const std::string FLAG_ENCODER_SHORT = "-e";

        const std::string FLAG_INPUT_LONG  = "--problem-file";
        const std::string FLAG_OUTPUT_LONG = "--output-dir";
        const std::string FLAG_SEARCH_LONG = "--search";
        const std::string FLAG_ENCODER_LONG = "--encoder";
        
        argparse::ArgumentParser& create_up_cli_argparser(void);
    }
}

