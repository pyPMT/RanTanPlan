cmake_minimum_required(VERSION 3.20)
project(pmt CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find staged dependencies (superbuild passes CMAKE_PREFIX_PATH)

# For now but we need to find a way to fix this.
find_package(Protobuf CONFIG REQUIRED)
find_package(Z3       CONFIG REQUIRED)
find_package(spdlog   CONFIG REQUIRED)
find_package(xplugin         REQUIRED)

# Create the protobuf files.
set(PROTO_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/proto/unified_planning.proto
)

# Generate into the build tree
set(GEN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/generated_files)
file(MAKE_DIRECTORY "${GEN_DIR}")

# Run protoc independently of any target, and collect outputs
protobuf_generate(
  LANGUAGE cpp
  PROTOS ${PROTO_FILES}
  PROTOC_OUT_DIR "${GEN_DIR}"
  OUT_VAR GEN_SRCS          # -> list of .pb.cc / .pb.h files
)

# Make codegen part of the default build; it materializes GEN_SRCS
add_custom_target(proto_codegen ALL
  DEPENDS ${GEN_SRCS}
)

set(TASK_SRCS src/task/task.cpp
              src/task/smt_task.cpp)
set(TASK_HEADERS src/task/task.hpp 
                 src/task/smt_task.hpp 
                 src/task/protobuf_aliases.hpp
                 src/task/elements/type.hpp
                 src/task/elements/object.hpp)


set(ENCODERS_SRCS    src/encoders/sequential.cpp)

set(PLANNERS_SRCS    src/planners/single.cpp)

set(PROPAGATORS_SRCS src/propagators/null_propagator.cpp)

set(PLUGIN_HEADERS src/plugin_bases/encoder_plugin_base.hpp
                   src/plugin_bases/planner_plugin_base.hpp
                   src/plugin_bases/propagator_plugin_base.hpp)

set(PLAN_HEADERS src/plan/plan.hpp)
set(PLAN_SRCS    src/plan/plan.cpp)

set(UTILS_SRCS src/utils/cli_argparsers.cpp)
set(UTILS_HEADERS src/utils/cli_argparsers.hpp
                  src/utils/templates.hpp)

add_library(pmtlib STATIC ${TASK_SRCS} ${UTILS_SRCS} ${ENCODERS_SRCS} ${PLANNERS_SRCS} ${PROPAGATORS_SRCS} ${GEN_SRCS} 
                          ${TASK_HEADERS} ${UTILS_HEADERS} ${PLUGIN_HEADERS})
add_dependencies(pmtlib proto_codegen)

target_link_libraries(pmtlib
  PUBLIC
    protobuf::libprotobuf
    spdlog::spdlog
    ${Z3_LIBRARIES}
    ${XPLUGIN_XPLUGIN_LIBRARY}
)

target_include_directories(pmtlib
  PUBLIC
    ${CMAKE_CURRENT_BINARY_DIR}/generated
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_INSTALL_PREFIX}/include
    ${GEN_DIR}
)

# Example executable
add_executable(planmt-up-cli src/cli/planmt-up-cli.cpp)
target_link_libraries(planmt-up-cli PRIVATE pmtlib)
target_link_libraries(planmt-up-cli PUBLIC spdlog::spdlog)

target_include_directories(planmt-up-cli PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(planmt-up-cli PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_include_directories(planmt-up-cli PRIVATE ${CMAKE_INSTALL_PREFIX}/include)
target_include_directories(planmt-up-cli PRIVATE ${GEN_DIR})

# (Optional) Install your lib/pkg into the same stage
include(GNUInstallDirs)
install(TARGETS pmtlib planmt-up-cli
  EXPORT pmtlibTargets
  RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
