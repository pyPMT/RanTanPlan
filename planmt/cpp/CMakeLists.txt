# cmake_minimum_required(VERSION 3.10)
cmake_minimum_required(VERSION 3.30)

# ==============================================================================
# PROJECT CONFIGURATION
# ==============================================================================
project(planmt)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set default output directory
if(NOT DEFINED OUTPUT_DIR)
    set(OUTPUT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../bin" CACHE STRING "Output directory for executable")
endif()

# ==============================================================================
# PROTOBUF SETUP
# ==============================================================================
# https://github.com/protocolbuffers/protobuf/issues/12637
if(APPLE)
    find_package(Protobuf REQUIRED CONFIG)
else()
    find_package(Protobuf REQUIRED)
endif()
# Abseil (Google's C++ common libraries) for protobuf
#find_package(absl REQUIRED)
# no longer needed, protobuf now uses absl internally

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
# TARGET DEFINITION
# ==============================================================================

set(ABSTRACTION_SRCS src/abstraction/headers/achievers_analysis.h 
                     src/abstraction/src/achievers_analysis.cpp)

set(ARPG_SRCS src/arpg/headers/supporter.h 
              src/arpg/headers/arpg.h 
              src/arpg/headers/interval.h
              src/arpg/headers/relaxed_state.h
              src/arpg/src/arpg.cpp
              src/arpg/src/supporter.cpp)

set(CONFIG_SRCS src/config/headers/config.h
                src/config/headers/cli_parser.h
                src/config/src/config.cpp
                src/config/src/cli_parser.cpp)

set(ENCODERS_SRCS 
src/encoders/headers/parallelism/eager_interference_analysis.h
src/encoders/headers/parallelism/exists_semantics.h
src/encoders/headers/parallelism/forall_semantics.h
src/encoders/headers/parallelism/graph.h
src/encoders/headers/parallelism/interference_analysis_factory.h
src/encoders/headers/parallelism/interference_analysis.h
src/encoders/headers/parallelism/interference_analyzer.h
src/encoders/headers/parallelism/lazy_interference_analysis.h
src/encoders/headers/parallelism/parallelism_factory.h
src/encoders/headers/parallelism/parallelism_strategy.h
src/encoders/headers/parallelism/semantic_interference_analysis.h
src/encoders/headers/parallelism/sequential_semantics.h
src/encoders/headers/base_encoder.h
src/encoders/headers/chained_grounded_encoder.h
src/encoders/headers/encoder_factory.h
src/encoders/headers/grounded_encoder.h
src/encoders/headers/grounded_encoding_visitor.h
src/encoders/headers/lifted_encoding_visitor.h
src/encoders/headers/r2e_grounded_encoder.h
src/encoders/headers/reified_grounded_encoder.h
src/encoders/headers/z3_variable_factory.h
src/encoders/src/parallelism/eager_interference_analysis.cpp
src/encoders/src/parallelism/exists_semantics.cpp
src/encoders/src/parallelism/forall_semantics.cpp
src/encoders/src/parallelism/graph.cpp
src/encoders/src/parallelism/interference_analysis_factory.cpp
src/encoders/src/parallelism/interference_analysis.cpp
src/encoders/src/parallelism/interference_analyzer.cpp
src/encoders/src/parallelism/lazy_interference_analysis.cpp
src/encoders/src/parallelism/parallelism_factory.cpp
src/encoders/src/parallelism/semantic_interference_analysis.cpp
src/encoders/src/parallelism/sequential_semantics.cpp
src/encoders/src/chained_grounded_encoder.cpp
src/encoders/src/encoder_factory.cpp
src/encoders/src/grounded_encoder.cpp
src/encoders/src/grounded_encoding_visitor.cpp
src/encoders/src/lifted_encoding_visitor.cpp
src/encoders/src/r2e_grounded_encoder.cpp
src/encoders/src/reified_grounded_encoder.cpp
src/encoders/src/z3_variable_factory.cpp
)

set(PLANNERS_SRCS 
src/planners/headers/propagators/decision_heuristic_propagator.h
src/planners/headers/propagators/exists_propagator.h
src/planners/headers/propagators/forall_propagator.h
src/planners/headers/propagators/lazy_forall_propagator.h
src/planners/headers/propagators/null_propagator.h
src/planners/headers/propagators/propagator_factory.h
src/planners/headers/propagators/propagator_strategy.h
src/planners/headers/propagators/propagator_types.h
src/planners/headers/diverse.h
src/planners/headers/sequential.h
src/planners/headers/single.h
src/planners/headers/topk.h
src/planners/headers/topq.h
src/planners/src/propagators/decision_heuristic_propagator.cpp
src/planners/src/propagators/exists_propagator.cpp
src/planners/src/propagators/forall_propagator.cpp
src/planners/src/propagators/lazy_forall_propagator.cpp
src/planners/src/propagators/propagator_factory.cpp
src/planners/src/diverse.cpp
src/planners/src/sequential.cpp
src/planners/src/single.cpp
src/planners/src/topk.cpp
src/planners/src/topq.cpp
)

set(PROBLEM_SRCS
src/problem/headers/visitors/expression_visitor.h
src/problem/headers/visitors/fluent_collector.h
src/problem/headers/visitors/fluent_polarity_collector.h
src/problem/headers/visitors/print_visitor.h
src/problem/headers/visitors/variable_collector.h
src/problem/headers/action.h
src/problem/headers/assignment.h
src/problem/headers/atom.h
src/problem/headers/effect_expression.h
src/problem/headers/effect.h
src/problem/headers/expression.h
src/problem/headers/fluent.h
src/problem/headers/goal.h
src/problem/headers/object.h
src/problem/headers/parameter.h
src/problem/headers/plan.h
src/problem/headers/problem.h
src/problem/headers/protobuf_aliases.h
src/problem/headers/real.h
src/problem/headers/type.h
src/problem/src/visitors/expression_visitor.cpp
src/problem/src/action.cpp
src/problem/src/assignment.cpp
src/problem/src/atom.cpp
src/problem/src/effect_expression.cpp
src/problem/src/effect.cpp
src/problem/src/expression.cpp
src/problem/src/fluent.cpp
src/problem/src/goal.cpp
src/problem/src/object.cpp
src/problem/src/parameter.cpp
src/problem/src/plan.cpp
src/problem/src/problem.cpp
src/problem/src/real.cpp
src/problem/src/type.cpp
)

set(SYMMETRIES_SRCS 
src/symmetries/headers/smt_symmetry_checker.h
src/symmetries/src/smt_symmetry_checker.cpp
)

set(UTIL_SRCS 
src/util/headers/memory_tracker.h
src/util/headers/stats.h
src/util/src/memory_tracker.cpp
src/util/src/stats.cpp
)

add_library(pmtlib STATIC 
    ${ABSTRACTION_SRCS}
    ${ARPG_SRCS}
    ${CONFIG_SRCS}
    ${PROBLEM_SRCS}
    ${ENCODERS_SRCS}
    ${PLANNERS_SRCS}
    ${SYMMETRIES_SRCS}
    ${UTIL_SRCS}
)


set(PROTO_OUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated)
# Generate protobuf for pmtlib and put outputs in PROTO_OUT_DIR
set(PROTO_FILES ${CMAKE_CURRENT_SOURCE_DIR}/proto/unified_planning.proto)
# message(STATUS "---------------> PROTO_FILES: ${PROTO_FILES}")
protobuf_generate(
    TARGET   pmtlib
    PROTOS   ${PROTO_FILES}
    LANGUAGE cpp
    PROTOC_OUT_DIR ${PROTO_OUT_DIR}
)
# pmtlib must see the generated headers to compile its own sources
target_include_directories(pmtlib PRIVATE ${CMAKE_CURRENT_BINARY_DIR} ${PROTO_OUT_DIR}/proto ${Z3_INCLUDE_DIR})

# And link protobuf to the target that owns the generated files
target_link_libraries(pmtlib PUBLIC protobuf::libprotobuf ${Z3_LINK_LIBRARY})
# target_include_directories(pmtlib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/proto)

target_include_directories(pmtlib PUBLIC 
${CMAKE_CURRENT_SOURCE_DIR}/src
${CMAKE_CURRENT_SOURCE_DIR}/src/abstraction/headers
${CMAKE_CURRENT_SOURCE_DIR}/src/arpg/headers
${CMAKE_CURRENT_SOURCE_DIR}/src/config/headers
${CMAKE_CURRENT_SOURCE_DIR}/src/problem/headers
${CMAKE_CURRENT_SOURCE_DIR}/src/encoders/headers
${CMAKE_CURRENT_SOURCE_DIR}/src/planners/headers
${CMAKE_CURRENT_SOURCE_DIR}/src/symmetries/headers
${CMAKE_CURRENT_SOURCE_DIR}/src/util/headers
${PROTO_OUT_DIR}
)


# ==============================================================================
# TARGET CONFIGURATION
# ==============================================================================
# Include directories
target_include_directories(pmtlib INTERFACE
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
    ${CMAKE_CURRENT_SOURCE_DIR}/src/abstraction/     # abstraction headers
    ${Protobuf_INCLUDE_DIRS}                         # protobuf lib headers
                                    # Z3 headers
)


# Create main executable
add_executable(planmt src/main.cpp)
target_link_libraries(planmt PRIVATE pmtlib)
target_include_directories(planmt PRIVATE ${PROTO_OUT_DIR}/proto)



# ==============================================================================
# PLATFORM-SPECIFIC FIXES
# ==============================================================================
# Fix Z3 library reference on macOS
if(APPLE AND NOT Z3_FOUND)
    # This applies only when Z3 is found manually (NOT Z3_FOUND is true).
    # Z3_LIBRARY holds the full path to libz3.dylib.
    # Z3_LIB_DIR holds the directory containing libz3.dylib.

    # Instead of relying on RPATH, use install_name_tool to change the recorded
    # dependency path in the 'planmt' executable from 'libz3.dylib'
    # to the absolute path ${Z3_LIBRARY}.
    message(STATUS "macOS manual Z3: Using install_name_tool to set absolute path for libz3.dylib in 'planmt'")
    add_custom_command(TARGET planmt POST_BUILD
        COMMAND ${CMAKE_INSTALL_NAME_TOOL} -change libz3.dylib ${Z3_LIBRARY} $<TARGET_FILE:planmt>
        COMMENT "Fixing Z3 dylib reference in planmt executable to absolute path"
        VERBATIM
    )
endif()

# ==============================================================================
# INSTALLATION
# ==============================================================================
install(TARGETS planmt DESTINATION ${OUTPUT_DIR})

# ==============================================================================
# Set RPATH for Z3 if found manually
# ==============================================================================
if(NOT Z3_FOUND AND Z3_LIB_DIR)
    set_target_properties(planmt PROPERTIES
        BUILD_WITH_INSTALL_RPATH TRUE
        INSTALL_RPATH "${Z3_LIB_DIR}"
        SKIP_BUILD_RPATH FALSE
        SKIP_INSTALL_RPATH FALSE
    )
endif()