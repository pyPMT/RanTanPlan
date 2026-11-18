cmake_minimum_required(VERSION 3.15)

# ==============================================================================
# PROJECT CONFIGURATION
# ==============================================================================
project(rantanplan)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Emit compile_commands.json so clangd/editor tooling knows the real include
# paths and flags (Z3, nanobind, -std=c++20) instead of guessing.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ==============================================================================
# PYTHON / NANOBIND SETUP
# ==============================================================================
# scikit-build-core passes interpreter hints for the building Python; for a
# plain CMake configure (e.g. an IDE), FindPython picks the interpreter from
# the activated shell, or pass -DPython_EXECUTABLE=... explicitly.
find_package(Python 3.8 COMPONENTS Interpreter Development.Module REQUIRED)

# nanobind comes from its pip package in the build environment; -Dnanobind_DIR
# (or -Dnanobind_ROOT) remains as an escape hatch.
if(NOT DEFINED nanobind_DIR AND NOT DEFINED nanobind_ROOT)
    execute_process(
        COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
        OUTPUT_VARIABLE nanobind_ROOT
        OUTPUT_STRIP_TRAILING_WHITESPACE
        RESULT_VARIABLE NANOBIND_LOOKUP_RESULT
    )
    if(NOT NANOBIND_LOOKUP_RESULT EQUAL 0)
        message(FATAL_ERROR "Could not locate nanobind via ${Python_EXECUTABLE} "
                            "(is the nanobind pip package installed?)")
    endif()
endif()
find_package(nanobind CONFIG REQUIRED)

# ==============================================================================
# Z3 SOLVER SETUP
# ==============================================================================
# The module must link the exact libz3 the z3 pip package loads (two copies in
# one process is undefined behavior), so Z3 is located via the interpreter's
# z3 package. -DZ3_INCLUDE_DIR/-DZ3_LIB_DIR remain as escape hatches.
if(NOT DEFINED Z3_INCLUDE_DIR OR NOT DEFINED Z3_LIB_DIR)
    execute_process(
        COMMAND "${Python_EXECUTABLE}" -c "import z3, os; print(os.path.dirname(z3.__file__))"
        OUTPUT_VARIABLE Z3_PACKAGE_DIR
        OUTPUT_STRIP_TRAILING_WHITESPACE
        RESULT_VARIABLE Z3_LOOKUP_RESULT
    )
    if(NOT Z3_LOOKUP_RESULT EQUAL 0)
        message(FATAL_ERROR "Could not locate the z3 package via ${Python_EXECUTABLE} "
                            "(is the z3-solver pip package installed?)")
    endif()
    set(Z3_INCLUDE_DIR "${Z3_PACKAGE_DIR}/include")
    set(Z3_LIB_DIR "${Z3_PACKAGE_DIR}/lib")
endif()

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

# ==============================================================================
# TARGET DEFINITION
# ==============================================================================
# Core static library. The nanobind module links this target and inherits
# its usage requirements (includes, Z3) via PUBLIC.
add_library(rantanplan_core STATIC
    src/api/solver_api.cpp
    src/api/ir_io.cpp
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
    src/planners/formula_exporter.cpp
    src/encoders/abstract_suffix_encoder.cpp
    src/planners/propagators/propagator_strategy.cpp
    src/planners/propagators/modules/logging_module.cpp
    src/planners/propagators/modules/exists_cycle_module.cpp
    src/planners/propagators/modules/forall_mutex_module.cpp
    src/planners/propagators/modules/frame_axiom_module.cpp
    src/symmetries/smt_symmetry_checker.cpp
    src/analysis/achievers_analysis.cpp
    src/analysis/numeric_relaxed_planning_graph.cpp
    src/analysis/graph.cpp
    src/analysis/interference_analysis.cpp
    src/analysis/eager_interference_analysis.cpp
    src/analysis/eager_semantic_interference_analysis.cpp
    src/analysis/lazy_interference_analysis.cpp
    src/analysis/semantic_interference_predicate.cpp
    src/analysis/semantic_interference_analysis.cpp
    src/analysis/numeric_constraint_analyzer.cpp
    src/preprocessing/preprocessed_problem.cpp
    src/preprocessing/preprocessor_options.cpp
    src/passes/pipeline.cpp
    src/passes/arithmetic_profile_pass.cpp
    src/passes/action_pruning_pass.cpp
    src/passes/mutex_analysis_pass.cpp
    src/passes/landmark_analysis_pass.cpp
    src/passes/bound_tightening.cpp
    src/passes/h2_mutex_checker.cpp
    src/passes/symmetry_analysis_pass.cpp
    src/passes/int_parameter_instantiation_pass.cpp
    src/passes/array_lowering_pass.cpp
    src/passes/set_lowering_pass.cpp
    src/passes/term_resolution_pass.cpp
    src/passes/grounding_pass.cpp
    src/passes/quantifier_expansion_pass.cpp
    src/passes/cwa_initial_state_pass.cpp
    src/passes/static_fluent_pass.cpp
    src/passes/write_only_fluent_pass.cpp
    src/passes/effect_normalization_pass.cpp
    src/passes/cnf_normalization_pass.cpp
    src/problem/substitution.cpp
    src/grounding/fact_index.cpp
    src/grounding/binding_matcher.cpp
    src/grounding/action_instantiator.cpp
    src/grounding/reachability_grounder.cpp
    src/grounding/array_lowering.cpp
    src/grounding/numeric_bounds_index.cpp
    src/grounding/object_fluent_index.cpp
    src/grounding/interval_evaluator.cpp
)

# ==============================================================================
# TARGET CONFIGURATION
# ==============================================================================
# Include directories (PUBLIC: the core's headers reference these paths, so
# consumers of the library need them too)
target_include_directories(rantanplan_core PUBLIC
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

# Link libraries (PUBLIC: Z3 types appear in the core's public headers)
target_link_libraries(rantanplan_core PUBLIC
    ${Z3_LINK_LIBRARY} # Ensure this uses the full path from Z3_LIBRARY
)

# Python extension module — the only artifact. The static core links into a
# shared module, so it must be position-independent.
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
    message(STATUS "macOS manual Z3: Using install_name_tool to set absolute path for libz3.dylib in '_rantanplan'")
    add_custom_command(TARGET _rantanplan POST_BUILD
        COMMAND ${CMAKE_INSTALL_NAME_TOOL} -change libz3.dylib ${Z3_LIBRARY} $<TARGET_FILE:_rantanplan>
        COMMENT "Fixing Z3 dylib reference in _rantanplan module to absolute path"
        VERBATIM
    )
endif()

# ==============================================================================
# INSTALLATION
# ==============================================================================
# scikit-build-core stages the install tree into the wheel; the module lands
# inside the rantanplan package (for editable installs it is resolved from the
# build tree via the editable redirect).
install(TARGETS _rantanplan LIBRARY DESTINATION rantanplan)

# ==============================================================================
# Set RPATH for Z3 if found manually
# ==============================================================================
if(NOT Z3_FOUND AND Z3_LIB_DIR)
    set_target_properties(_rantanplan PROPERTIES
        BUILD_WITH_INSTALL_RPATH TRUE
        INSTALL_RPATH "${Z3_LIB_DIR}"
        SKIP_BUILD_RPATH FALSE
        SKIP_INSTALL_RPATH FALSE
    )
endif()