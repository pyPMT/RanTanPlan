cmake_minimum_required(VERSION 3.20)
project(pmt CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find staged dependencies (superbuild passes CMAKE_PREFIX_PATH)
find_package(Protobuf CONFIG REQUIRED)
find_package(Z3       CONFIG REQUIRED)
find_package(ng-log   CONFIG REQUIRED)
find_package(glog     CONFIG REQUIRED)

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
