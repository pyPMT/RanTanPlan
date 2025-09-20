
#include "pmt/version.hpp"
#include "cli_argparsers.hpp"



namespace planmt {
    namespace utils {
        argparse::ArgumentParser& create_up_cli_argparser(void) {
            auto program_name = std::string("pmt-up-cli");
            auto description = std::string("PMT Unified Planning Command Line Interface: ") +
                               std::string("A UP Interface for solving planning problems using the PMT library.") + 
                               std::string("Version ") + PMT_VERSION_STRING;

            static argparse::ArgumentParser parser(program_name, description);
            
            // I think the arguments should be like this:
            // pmt-up-cli  --search "<FD style>" --encoder "<FD style>" --problem-file <input_problem.pb> --output-dir <dump_dir>
            parser.add_argument(planmt::utils::FLAG_INPUT_SHORT, planmt::utils::FLAG_INPUT_LONG)
                  .help("Path to the input problem file in protobuf format") 
                  .required()
                  .metavar("FILE");
            
            parser.add_argument(planmt::utils::FLAG_OUTPUT_SHORT, planmt::utils::FLAG_OUTPUT_LONG)
                  .help("Directory to save the output solution file")
                  .default_value(std::string("."))
                  .metavar("DIR");

            parser.add_argument(planmt::utils::FLAG_SEARCH_SHORT, planmt::utils::FLAG_SEARCH_LONG)
                  .help("Search strategy to use (e.g., 'topk(timeout=300, memory=2480, k=3)')")
                  .required()
                  .metavar("SEARCH");

            parser.add_argument(planmt::utils::FLAG_ENCODER_SHORT, planmt::utils::FLAG_ENCODER_LONG)
                  .help("Encoder to use (e.g., 'sequential(detect_symmetries=false)')")
                  .required()
                  .metavar("ENCODER");

            return parser;
        }
    }
}
