cmake_minimum_required(VERSION 3.14)

# ==============================================================================
# PROJECT CONFIGURATION
# ==============================================================================
project(rantanplan)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set default output directory
if(NOT DEFINED OUTPUT_DIR)
    set(OUTPUT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../bin" CACHE STRING "Output directory for executable")
endif()

# ==============================================================================
# PROTOBUF SETUP
# ==============================================================================
option(FETCH_PROTOBUF "Force fetching protobuf via FetchContent instead of using system protobuf" OFF)

# Try system protobuf first (unless forced to fetch)
if(NOT FETCH_PROTOBUF)
    # https://github.com/protocolbuffers/protobuf/issues/12637
    if(APPLE)
        find_package(Protobuf QUIET CONFIG)
    else()
        find_package(Protobuf QUIET)
    endif()
endif()

if(Protobuf_FOUND OR PROTOBUF_FOUND)
    message(STATUS "Found system protobuf: ${Protobuf_VERSION}")
else()
    message(STATUS "System protobuf not found, fetching via FetchContent...")
    include(FetchContent)

    # Configure protobuf build options before fetching
    set(protobuf_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(protobuf_BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
    set(protobuf_INSTALL OFF CACHE BOOL "" FORCE)
    set(protobuf_ABSL_PROVIDER "module" CACHE STRING "" FORCE)
    set(ABSL_PROPAGATE_CXX_STD ON CACHE BOOL "" FORCE)
    set(ABSL_BUILD_TESTING OFF CACHE BOOL "" FORCE)

    FetchContent_Declare(
        protobuf
        GIT_REPOSITORY https://github.com/protocolbuffers/protobuf.git
        GIT_TAG        v25.6
        GIT_SHALLOW    TRUE
        GIT_SUBMODULES "third_party/abseil-cpp"
    )
    FetchContent_MakeAvailable(protobuf)

    # protobuf_generate is not automatically available via FetchContent,
    # include it explicitly from the fetched protobuf source
    include(${protobuf_SOURCE_DIR}/cmake/protobuf-generate.cmake)
endif()

# ==============================================================================
# Z3 SOLVER SETUP
# ==============================================================================
# Try to find Z3 using modern CMake first
find_package(Z3 QUIET)

if(Z3_FOUND)
    message(STATUS "Found Z3 using CMake package config")
    set(Z3_LINK_LIBRARY z3::libz3)
else()
    # Fallback to manual Z3 setup (original approach)
    message(STATUS "Using manual Z3 setup")
    
    # Z3 paths are set by setup.py during Python package installation
    set(Z3_INCLUDE_DIR ${Z3_INCLUDE_DIR} CACHE STRING "Z3 include directory")
    set(Z3_LIB_DIR ${Z3_LIB_DIR} CACHE STRING "Z3 library directory")

    # Locate Z3 library
    find_library(Z3_LIBRARY
        NAMES z3 libz3
        PATHS ${Z3_LIB_DIR}
        NO_DEFAULT_PATH
    )

    if(NOT Z3_LIBRARY)
        message(FATAL_ERROR "Z3 library not found in ${Z3_LIB_DIR}")
    endif()

    message(STATUS "Found Z3 library: ${Z3_LIBRARY}")
    set(Z3_LINK_LIBRARY ${Z3_LIBRARY}) # Use the full path to the library
endif()


# ==============================================================================
# PYTHON / NANOBIND SETUP
# ==============================================================================
# Both located via the active venv: build.py passes Python_EXECUTABLE (pins
# the interpreter whose ABI the module targets) and nanobind_DIR (from the
# nanobind pip package, like Z3).
find_package(Python 3.8 COMPONENTS Interpreter Development.Module REQUIRED)
find_package(nanobind CONFIG REQUIRED)

# ==============================================================================
# TARGET DEFINITION
# ==============================================================================
# Core static library: everything except the CLI driver. The executable and
# the nanobind module link this target and inherit its usage requirements
# (includes, protobuf, Z3) via PUBLIC.
add_library(rantanplan_core STATIC
    src/api/solver_api.cpp
    src/config/config.cpp
    src/config/cli_parser.cpp
    src/config/strategy_registry.cpp
    src/api/strategy_factory.cpp
    src/util/memory_tracker.cpp
    src/util/stats.cpp
    src/util/scoped_timer.cpp
    src/util/logger.cpp
    src/problem/real.cpp
    src/problem/atom.cpp
    src/problem/parameter.cpp
    src/problem/expr_pool.cpp
    src/problem/effect_expression.cpp
    src/problem/effect.cpp
    src/problem/object.cpp
    src/problem/fluent.cpp
    src/problem/goal.cpp
    src/problem/assignment.cpp
    src/problem/action.cpp
    src/problem/plan.cpp
    src/problem/problem.cpp
    src/problem/protobuf_io.cpp
    src/problem/type.cpp
    src/problem/visitors/fluent_collector.cpp
    src/problem/visitors/fluent_polarity_collector.cpp
    src/z3core/lifted_encoding_visitor.cpp
    src/z3core/grounded_encoding_visitor.cpp
    src/encoders/grounded_encoder.cpp
    src/encoders/chained_grounded_encoder.cpp
    src/encoders/r2e_grounded_encoder.cpp
    src/encoders/parallelism/sequential_semantics.cpp
    src/encoders/parallelism/forall_semantics.cpp
    src/encoders/parallelism/exists_semantics.cpp
    src/z3core/z3_variable_factory.cpp
    src/planners/base_planner.cpp
    src/planners/incremental_horizon_planner.cpp
    src/planners/double_tail_planner.cpp
    src/planners/branch_and_bound_planner.cpp
    src/planners/pdla_planner.cpp
    src/planners/formula_exporter.cpp
    src/encoders/abstract_suffix_encoder.cpp
    src/planners/propagators/propagator_strategy.cpp
    src/planners/propagators/modules/logging_module.cpp
    src/planners/propagators/modules/exists_cycle_module.cpp
    src/planners/propagators/modules/forall_mutex_module.cpp
    src/planners/propagators/modules/frame_axiom_module.cpp
    src/symmetries/smt_symmetry_checker.cpp
    src/analysis/achievers_analysis.cpp
    src/analysis/relaxed_planning_graph.cpp
    src/analysis/numeric_relaxed_planning_graph.cpp
    src/analysis/graph.cpp
    src/analysis/interference_analysis.cpp
    src/analysis/eager_interference_analysis.cpp
    src/analysis/eager_semantic_interference_analysis.cpp
    src/analysis/lazy_interference_analysis.cpp
    src/analysis/semantic_interference_analysis.cpp
    src/analysis/numeric_constraint_analyzer.cpp
    src/preprocessing/preprocessed_problem.cpp
    src/preprocessing/preprocessor_options.cpp
    src/passes/pipeline.cpp
    src/passes/arithmetic_profile_pass.cpp
    src/passes/action_pruning_pass.cpp
    src/passes/invariant_oracle_pass.cpp
    src/passes/bound_tightening.cpp
    src/passes/h2_mutex_checker.cpp
    src/passes/symmetry_analysis_pass.cpp
    src/passes/grounding_pass.cpp
    src/passes/cwa_initial_state_pass.cpp
    src/passes/static_fluent_pass.cpp
    src/problem/substitution.cpp
    src/grounding/fact_index.cpp
    src/grounding/binding_matcher.cpp
    src/grounding/action_instantiator.cpp
    src/grounding/reachability_grounder.cpp
    src/grounding/numeric_bounds_index.cpp
    src/grounding/object_fluent_index.cpp
    src/grounding/interval_evaluator.cpp
)

# Define proto files and generate C++ sources
set(PROTO_FILES ${CMAKE_CURRENT_SOURCE_DIR}/proto/unified_planning.proto)
message(STATUS "PROTO_FILES: ${PROTO_FILES}")
# generate protobuf files for the core target
protobuf_generate(TARGET rantanplan_core
    PROTOS ${PROTO_FILES}
    LANGUAGE cpp
)

# ==============================================================================
# TARGET CONFIGURATION
# ==============================================================================
# Include directories (PUBLIC: the core's headers reference these paths, so
# consumers of the library need them too)
target_include_directories(rantanplan_core PUBLIC
    ${CMAKE_CURRENT_BINARY_DIR}                      # generated protobuf sources
    ${CMAKE_CURRENT_BINARY_DIR}/proto                # generated protobuf headers
    ${CMAKE_CURRENT_SOURCE_DIR}/src                  # our source headers
    ${CMAKE_CURRENT_SOURCE_DIR}/src/config           # config headers
    ${CMAKE_CURRENT_SOURCE_DIR}/src/util             # utility headers
    ${CMAKE_CURRENT_SOURCE_DIR}/src/problem          # problem representation headers
    ${CMAKE_CURRENT_SOURCE_DIR}/src/problem/visitors # visitor headers
    ${CMAKE_CURRENT_SOURCE_DIR}/src/encoders/        # encoders headers
    ${CMAKE_CURRENT_SOURCE_DIR}/src/planners/        # planners headers
    ${CMAKE_CURRENT_SOURCE_DIR}/src/symmetries/      # symmetries headers
    ${CMAKE_CURRENT_SOURCE_DIR}/src/analysis/        # analysis headers
    ${CMAKE_CURRENT_SOURCE_DIR}/src/passes/          # pass system headers
    ${CMAKE_CURRENT_SOURCE_DIR}/src/grounding/       # grounding headers
    ${CMAKE_CURRENT_SOURCE_DIR}/src/z3core/          # ExprID->Z3 translation layer
    ${CMAKE_CURRENT_SOURCE_DIR}/src/preprocessing/   # preprocessor contract headers
    ${Z3_INCLUDE_DIR}                                # Z3 headers
)

# Link libraries (PUBLIC: protobuf and Z3 types appear in the core's public
# headers — e.g. solver_api.hpp returns PlanGenerationResult)
target_link_libraries(rantanplan_core PUBLIC
    protobuf::libprotobuf
    ${Z3_LINK_LIBRARY} # Ensure this uses the full path from Z3_LIBRARY
)

# CLI executable: thin driver over the core library
add_executable(rantanplan src/main.cpp)
target_link_libraries(rantanplan PRIVATE rantanplan_core)

# Cross-run state regression harness: two preprocess+solve cycles in one
# process (nanobind migration, Phase 2). Built but not installed.
add_executable(double_solve_check tools/double_solve_check.cpp)
target_link_libraries(double_solve_check PRIVATE rantanplan_core)

# Python extension module. The static core links into a shared module, so it
# must be position-independent.
set_target_properties(rantanplan_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
nanobind_add_module(_rantanplan src/api/bindings.cpp)
target_link_libraries(_rantanplan PRIVATE rantanplan_core)

# ==============================================================================
# PLATFORM-SPECIFIC FIXES
# ==============================================================================
# Fix Z3 library reference on macOS
if(APPLE AND NOT Z3_FOUND)
    # This applies only when Z3 is found manually (NOT Z3_FOUND is true).
    # Z3_LIBRARY holds the full path to libz3.dylib.
    # Z3_LIB_DIR holds the directory containing libz3.dylib.

    # Instead of relying on RPATH, use install_name_tool to change the recorded
    # dependency path in the 'rantanplan' executable from 'libz3.dylib'
    # to the absolute path ${Z3_LIBRARY}.
    message(STATUS "macOS manual Z3: Using install_name_tool to set absolute path for libz3.dylib in 'rantanplan'")
    add_custom_command(TARGET rantanplan POST_BUILD
        COMMAND ${CMAKE_INSTALL_NAME_TOOL} -change libz3.dylib ${Z3_LIBRARY} $<TARGET_FILE:rantanplan>
        COMMENT "Fixing Z3 dylib reference in rantanplan executable to absolute path"
        VERBATIM
    )
    add_custom_command(TARGET double_solve_check POST_BUILD
        COMMAND ${CMAKE_INSTALL_NAME_TOOL} -change libz3.dylib ${Z3_LIBRARY} $<TARGET_FILE:double_solve_check>
        COMMENT "Fixing Z3 dylib reference in double_solve_check to absolute path"
        VERBATIM
    )
    add_custom_command(TARGET _rantanplan POST_BUILD
        COMMAND ${CMAKE_INSTALL_NAME_TOOL} -change libz3.dylib ${Z3_LIBRARY} $<TARGET_FILE:_rantanplan>
        COMMENT "Fixing Z3 dylib reference in _rantanplan module to absolute path"
        VERBATIM
    )
endif()

# ==============================================================================
# INSTALLATION
# ==============================================================================
install(TARGETS rantanplan DESTINATION ${OUTPUT_DIR})
# The Python module installs into the package directory (rantanplan/)
install(TARGETS _rantanplan LIBRARY DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/..)

# ==============================================================================
# Set RPATH for Z3 if found manually
# ==============================================================================
if(NOT Z3_FOUND AND Z3_LIB_DIR)
    set_target_properties(rantanplan double_solve_check _rantanplan PROPERTIES
        BUILD_WITH_INSTALL_RPATH TRUE
        INSTALL_RPATH "${Z3_LIB_DIR}"
        SKIP_BUILD_RPATH FALSE
        SKIP_INSTALL_RPATH FALSE
    )
endif()