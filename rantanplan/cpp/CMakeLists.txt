cmake_minimum_required(VERSION 3.15)

# ==============================================================================
# PROJECT CONFIGURATION
# ==============================================================================
project(rantanplan)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Emit compile_commands.json so clangd/editor tooling knows the real include
# paths and flags (cvc5, nanobind, -std=c++20) instead of guessing.
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
# CVC5 SOLVER SETUP
# ==============================================================================
# cvc5 is consumed straight out of a local build tree rather than an install
# prefix: cvc5 has no build-tree export(), so find_package would force a
# `cmake --install` after every cvc5 edit. Linking the shared libcvc5 in place
# means editing cvc5 needs only `make -C <cvc5>/build` -- no RantanPlan rebuild.
set(CVC5_SOURCE_DIR "$ENV{HOME}/git/cvc5" CACHE PATH
    "cvc5 source checkout to build against")
set(CVC5_BUILD_DIR "${CVC5_SOURCE_DIR}/build" CACHE PATH
    "cvc5 build tree (configure.sh default); consumed in place")

find_library(CVC5_LIBRARY
    NAMES cvc5
    PATHS "${CVC5_BUILD_DIR}/src"
    NO_DEFAULT_PATH
)

if(NOT CVC5_LIBRARY)
    message(FATAL_ERROR
        "cvc5 library not found in ${CVC5_BUILD_DIR}/src.\n"
        "Build the cvc5 fork first (see README.md, 'Installation'), or point "
        "-DCVC5_SOURCE_DIR=... at the checkout.")
endif()

# Public headers live in the source tree; cvc5_export.h and cvc5_proof_rule.h
# are generated into the build tree.
set(CVC5_INCLUDE_DIRS
    "${CVC5_SOURCE_DIR}/include"
    "${CVC5_BUILD_DIR}/include"
)
if(NOT EXISTS "${CVC5_SOURCE_DIR}/include/cvc5/cvc5.h")
    message(FATAL_ERROR "cvc5 headers not found in ${CVC5_SOURCE_DIR}/include")
endif()

message(STATUS "Found cvc5 library: ${CVC5_LIBRARY}")

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
    src/smtcore/term_builder.cpp
    src/smtcore/smt_variable_factory.cpp
    src/smtcore/lifted_encoding_visitor.cpp
    src/smtcore/grounded_encoding_visitor.cpp
    src/encoders/grounded_encoder.cpp
    src/encoders/parallelism/sequential_semantics.cpp
    src/encoders/parallelism/forall_semantics.cpp
    src/encoders/parallelism/exists_semantics.cpp
    src/planners/base_planner.cpp
    src/planners/incremental_horizon_planner.cpp
    src/planners/branch_and_bound_planner.cpp
    src/encoders/abstract_suffix_encoder.cpp
    # planners/propagators/ sources stay on disk but are OUT of the build:
    # the Z3 user-propagator subsystem is being reimplemented as in-solver
    # (THEORY_PLAN) reasoning inside the cvc5 fork. See migration-log.md.
    src/symmetries/smt_symmetry_checker.cpp
    src/analysis/achievers_analysis.cpp
    src/analysis/numeric_relaxed_planning_graph.cpp
    src/analysis/graph.cpp
    src/analysis/syntactic_interference.cpp
    src/analysis/eager_interference_analysis.cpp
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
    ${CMAKE_CURRENT_SOURCE_DIR}/src/smtcore/         # ExprID->SMT translation layer
    ${CMAKE_CURRENT_SOURCE_DIR}/src/preprocessing/   # preprocessor contract headers
    ${CVC5_INCLUDE_DIRS}                             # cvc5 headers
)

# Link libraries (PUBLIC: cvc5 types appear in the core's public headers)
target_link_libraries(rantanplan_core PUBLIC
    ${CVC5_LIBRARY}
)

# Python extension module — the only artifact. The static core links into a
# shared module, so it must be position-independent.
set_target_properties(rantanplan_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
nanobind_add_module(_rantanplan src/api/bindings.cpp)
target_link_libraries(_rantanplan PRIVATE rantanplan_core)

# ==============================================================================
# CVC5 FORK CONTRACT TEST (not built by default)
# ==============================================================================
# Regression test for the fork's THEORY_PLAN public API: PLAN_* kinds exist,
# carry the sort declared in their payload, round-trip their indices, reach
# the SAT solver, and the QF_*PLAN* logic strings parse. Build and run with:
#   cmake --build <dir> --target cvc5_theory_plan_smoke && <dir>/cvc5_theory_plan_smoke
add_executable(cvc5_theory_plan_smoke EXCLUDE_FROM_ALL tools/cvc5_theory_plan_smoke.cpp)
target_include_directories(cvc5_theory_plan_smoke PRIVATE ${CVC5_INCLUDE_DIRS})
target_link_libraries(cvc5_theory_plan_smoke PRIVATE ${CVC5_LIBRARY})
set_target_properties(cvc5_theory_plan_smoke PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "${CVC5_BUILD_DIR}/src"
)

# ==============================================================================
# INSTALLATION
# ==============================================================================
# scikit-build-core stages the install tree into the wheel; the module lands
# inside the rantanplan package (for editable installs it is resolved from the
# build tree via the editable redirect).
install(TARGETS _rantanplan LIBRARY DESTINATION rantanplan)

# ==============================================================================
# RPATH so the module finds libcvc5 in the fork's build tree
# ==============================================================================
# cvc5's install name is already @rpath/libcvc5.1.dylib, so it needs only an
# RPATH entry -- no install_name_tool fixup. libpoly and GMP are absolute-path
# dependencies of libcvc5 and need nothing here.
set_target_properties(_rantanplan PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "${CVC5_BUILD_DIR}/src"
    SKIP_BUILD_RPATH FALSE
    SKIP_INSTALL_RPATH FALSE
)