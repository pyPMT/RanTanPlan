cmake_minimum_required(VERSION 3.20)
project(pmt CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find staged dependencies (superbuild passes CMAKE_PREFIX_PATH)
find_package(Protobuf CONFIG REQUIRED)
find_package(Z3       CONFIG REQUIRED)

# argparse is header-only; depending on its install, it might export a target
# If not, you can still include_directories if needed, but modern packages export targets.

# Your library
set(PROBLEM_SRCS
src/problem/visitors/expression_visitor.h
src/problem/visitors/fluent_collector.h
src/problem/visitors/fluent_polarity_collector.h
src/problem/visitors/print_visitor.h
src/problem/visitors/variable_collector.h
src/problem/action.h
src/problem/assignment.h
src/problem/atom.h
src/problem/effect_expression.h
src/problem/effect.h
src/problem/expression.h
src/problem/fluent.h
src/problem/goal.h
src/problem/object.h
src/problem/parameter.h
src/problem/plan.h
src/problem/problem.h
src/problem/protobuf_aliases.h
src/problem/real.h
src/problem/type.h
src/problem/visitors/expression_visitor.cpp
src/problem/action.cpp
src/problem/assignment.cpp
src/problem/atom.cpp
src/problem/effect_expression.cpp
src/problem/effect.cpp
src/problem/expression.cpp
src/problem/fluent.cpp
src/problem/goal.cpp
src/problem/object.cpp
src/problem/parameter.cpp
src/problem/plan.cpp
src/problem/problem.cpp
src/problem/real.cpp
src/problem/type.cpp
)


add_library(pmtlib STATIC ${PROBLEM_SRCS})

# Generated protobuf files into binary tree
set(PROTO ${CMAKE_CURRENT_SOURCE_DIR}/proto/unified_planning.proto)

# If protobuf_generate is not auto-included by the package, uncomment this:
# include(${Protobuf_DIR}/protobuf-generate.cmake)

file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/generated)
protobuf_generate(
  TARGET pmtlib
  PROTOS ${PROTO}
  LANGUAGE cpp
  PROTOC_OUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated   # optional
)

target_link_libraries(pmtlib
  PUBLIC
    protobuf::libprotobuf
    z3::libz3
)

target_include_directories(pmtlib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${CMAKE_CURRENT_BINARY_DIR}/generated
)

# Example executable
add_executable(planmt src/main.cpp)
target_link_libraries(planmt PRIVATE pmtlib)

# (Optional) Install your lib/pkg into the same stage
include(GNUInstallDirs)
install(TARGETS pmtlib planmt
  EXPORT pmtlibTargets
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# # (Optional) Export a package config so consumers can find_package(pmt)
# include(CMakePackageConfigHelpers)
# write_basic_package_version_file(
#   "${CMAKE_CURRENT_BINARY_DIR}/pmtConfigVersion.cmake"
#   COMPATIBILITY SameMajorVersion
# )
# configure_package_config_file(
#   ${CMAKE_CURRENT_SOURCE_DIR}/cmake/pmtConfig.cmake.in
#   ${CMAKE_CURRENT_BINARY_DIR}/pmtConfig.cmake
#   INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/pmt
# )
# install(EXPORT pmtlibTargets
#   FILE pmtTargets.cmake
#   NAMESPACE pmt::
#   DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/pmt
# )
# install(FILES
#   "${CMAKE_CURRENT_BINARY_DIR}/pmtConfig.cmake"
#   "${CMAKE_CURRENT_BINARY_DIR}/pmtConfigVersion.cmake"
#   DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/pmt
# )




# cmake_minimum_required(VERSION 3.20)

# # ==============================================================================
# # PROJECT CONFIGURATION
# # ==============================================================================
# project(pmt)

# set(CMAKE_CXX_STANDARD 20)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

# # Set default output directory
# if(NOT DEFINED OUTPUT_DIR)
#     set(OUTPUT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../bin" CACHE STRING "Output directory for executable")
# endif()

# # ==============================================================================
# # Build external dependencies
# # ==============================================================================

# # https://github.com/protocolbuffers/protobuf/issues/12637
# # if(APPLE)
# #     find_package(Protobuf REQUIRED CONFIG)
# # else()
# #     find_package(Protobuf REQUIRED)
# # endif()

# include(ExternalProject)

# set(THIRD_PARTY_SRC "${CMAKE_BINARY_DIR}/third_party/src" CACHE PATH "Staging source dir")
# set(THIRD_PARTY_PREFIX "${CMAKE_BINARY_DIR}/third_party/install" CACHE PATH "Staging prefix")
# set(THIRD_PARTY_INCLUDE "${THIRD_PARTY_PREFIX}/include" CACHE PATH "Staging include dir")
# set(THIRD_PARTY_LIB "${THIRD_PARTY_PREFIX}/lib" CACHE PATH "Staging lib dir")

# file(MAKE_DIRECTORY "${THIRD_PARTY_PREFIX}")
# file(MAKE_DIRECTORY "${THIRD_PARTY_SRC}")
# file(MAKE_DIRECTORY "${THIRD_PARTY_INCLUDE}")
# file(MAKE_DIRECTORY "${THIRD_PARTY_LIB}")

# # Consistent build type if single-config generator
# if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
#   set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
# endif()

# # Helper to pass a consistent toolchain/build flags to subprojects
# set(COMMON_CMAKE_ARGS
#   -DCMAKE_INSTALL_PREFIX=${THIRD_PARTY_PREFIX}
#   -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
#   -DCMAKE_POSITION_INDEPENDENT_CODE=ON
# )

# ExternalProject_Add(ext_argparse
#   GIT_REPOSITORY https://github.com/p-ranav/argparse.git
#   GIT_TAG        v3.2           
#   GIT_SHALLOW    TRUE
#   UPDATE_DISCONNECTED TRUE
#   CMAKE_ARGS
#     ${COMMON_CMAKE_ARGS}
#     -DARGPARSE_BUILD_TESTS=OFF
#     # -DARGPARSE_INSTALL=OFF
#   INSTALL_DIR
#     ${THIRD_PARTY_PREFIX}
# )

# ExternalProject_Add(ext_ng_log
#   GIT_REPOSITORY https://github.com/ng-log/ng-log.git
#   GIT_TAG        v0.8.2           
#   GIT_SHALLOW    TRUE
#   UPDATE_DISCONNECTED TRUE
#   CMAKE_ARGS
#     ${COMMON_CMAKE_ARGS}
#     -DBUILD_EXAMPLES=OFF
#     -DWITH_GTEST=OFF
#     -DWITH_GFLAGS=OFF
#   INSTALL_DIR
#     ${THIRD_PARTY_PREFIX}
# )

# # TODO: At some point we will need to build protobuf ourselves and use its cmake.
# ExternalProject_Add(ext_protobuf
#   GIT_REPOSITORY https://github.com/protocolbuffers/protobuf.git
#   GIT_TAG        v32.0           
#   GIT_SHALLOW    TRUE
#   UPDATE_DISCONNECTED TRUE
#   CMAKE_ARGS
#     ${COMMON_CMAKE_ARGS}
#     -Dprotobuf_BUILD_TESTS=OFF
#     -Dprotobuf_BUILD_CONFORMANCE=OFF
#     -Dprotobuf_BUILD_EXAMPLES=OFF
#   INSTALL_DIR
#     ${THIRD_PARTY_PREFIX}
# )

# ExternalProject_Add(ext_z3
#   GIT_REPOSITORY https://github.com/Z3Prover/z3.git
#   GIT_TAG        z3-4.15.3
#   GIT_SHALLOW    TRUE
#   UPDATE_DISCONNECTED TRUE
#   CMAKE_ARGS
#     ${COMMON_CMAKE_ARGS}
#     -DZ3_ENABLE_EXAMPLE_TARGETS=OFF
#     -DZ3_INCLUDE_GIT_HASH=OFF
#     -DZ3_INCLUDE_GIT_DESCRIBE=OFF
#   INSTALL_DIR
#     ${THIRD_PARTY_PREFIX}
# )

# # ==============================================================================
# # Build PMT Library
# # ==============================================================================


# add_library(pmtlib STATIC
#     ${PROBLEM_SRCS}
# )

# # add_dependencies(ext_ng_log ext_protobuf)
# # add_dependencies(ext_argparse ext_ng_log)
# # add_dependencies(ext_z3 ext_argparse)

# add_dependencies(pmtlib ext_z3 ext_argparse ext_ng_log ext_protobuf)
# target_compile_features(pmtlib PUBLIC cxx_std_20)
# target_include_directories(pmtlib PRIVATE ${THIRD_PARTY_INCLUDE})


# set(PROTOC "${THIRD_PARTY_PREFIX}/bin/protoc")

# set(PROTO ${CMAKE_SOURCE_DIR}/proto/unified_planning.proto)
# set(GEN_DIR ${CMAKE_BINARY_DIR}/generated)
# file(MAKE_DIRECTORY ${GEN_DIR})

# add_custom_command(
#   OUTPUT ${GEN_DIR}/unified_planning.pb.cc ${GEN_DIR}/unified_planning.pb.h
#   COMMAND ${PROTOC}
#           --cpp_out=${GEN_DIR}
#           -I ${CMAKE_SOURCE_DIR}/proto
#           ${PROTO}
#   DEPENDS ${PROTO} ext_protobuf
#   COMMENT "Generating protobuf sources"
# )


# # # message(STATUS "---------------> PROTO_FILES: ${PROTO_FILES}")
# # protobuf_generate(
# #     TARGET   pmtlib
# #     PROTOS   ${CMAKE_CURRENT_SOURCE_DIR}/proto/unified_planning.proto
# #     LANGUAGE cpp
# #     PROTOC_OUT_DIR ${THIRD_PARTY_INCLUDE}
# # )

# # And link protobuf to the target that owns the generated files
# target_link_libraries(pmtlib PUBLIC 
#     ${THIRD_PARTY_LIB}/libprotobufd.a)
# # target_include_directories(pmtlib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/proto)

# include(GNUInstallDirs)  # gives standard dirs like lib/, include/, cmake/
# # Install the library binary
# install(TARGETS pmtlib
#     EXPORT pmtlibTargets
#     ARCHIVE DESTINATION ${THIRD_PARTY_LIB}   # .a / .lib
#     LIBRARY DESTINATION ${THIRD_PARTY_LIB}   # .so / .dylib
#     INCLUDES DESTINATION ${THIRD_PARTY_INCLUDE}
# )

# # Install the headers
# install(DIRECTORY include/ DESTINATION ${THIRD_PARTY_INCLUDE})




# # ==============================================================================
# # TARGET DEFINITION
# # ==============================================================================

# # Create main executable
# add_executable(planmt src/main.cpp)
# target_link_libraries(planmt PRIVATE pmtlib)
# target_include_directories(planmt PRIVATE ${THIRD_PARTY_INCLUDE})
