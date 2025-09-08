cmake_minimum_required(VERSION 3.20)
project(PMT_Superbuild)

include(ExternalProject)

# Where to stage third-party installs (headers/libs/cmake files)
set(STAGE_DIR "${CMAKE_BINARY_DIR}/stage" CACHE PATH "Superbuild install prefix")

# Build type for single-config generators
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

# Common args passed to all external CMake projects
set(COMMON_ARGS
  -DCMAKE_INSTALL_PREFIX=${STAGE_DIR}
  -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON
)

# ----------------------------
# External: argparse (header)
# ----------------------------
ExternalProject_Add(ext_argparse
  GIT_REPOSITORY https://github.com/p-ranav/argparse.git
  GIT_TAG        v3.2
  GIT_SHALLOW    TRUE
  UPDATE_DISCONNECTED TRUE
  CMAKE_ARGS
    ${COMMON_ARGS}
    -DARGPARSE_BUILD_TESTS=OFF
)

# ----------------------------
# External: ng-log
# ----------------------------
ExternalProject_Add(ext_ng_log
  GIT_REPOSITORY https://github.com/ng-log/ng-log.git
  GIT_TAG        v0.8.2
  GIT_SHALLOW    TRUE
  UPDATE_DISCONNECTED TRUE
  CMAKE_ARGS
    ${COMMON_ARGS}
    -DBUILD_EXAMPLES=OFF
    -DWITH_GTEST=OFF
    -DWITH_GFLAGS=OFF
)

# ----------------------------
# External: Protobuf
# ----------------------------
ExternalProject_Add(ext_protobuf
  GIT_REPOSITORY https://github.com/protocolbuffers/protobuf.git
  GIT_TAG        v32.0
  GIT_SHALLOW    TRUE
  UPDATE_DISCONNECTED TRUE
  CMAKE_ARGS
    ${COMMON_ARGS}
    -Dprotobuf_BUILD_TESTS=OFF
    -Dprotobuf_BUILD_CONFORMANCE=OFF
    -Dprotobuf_BUILD_EXAMPLES=OFF
)

# ----------------------------
# External: Z3
# ----------------------------
ExternalProject_Add(ext_z3
  GIT_REPOSITORY https://github.com/Z3Prover/z3.git
  GIT_TAG        z3-4.15.3
  GIT_SHALLOW    TRUE
  UPDATE_DISCONNECTED TRUE
  CMAKE_ARGS
    ${COMMON_ARGS}
    -DZ3_ENABLE_EXAMPLE_TARGETS=OFF
    -DZ3_INCLUDE_GIT_HASH=OFF
    -DZ3_INCLUDE_GIT_DESCRIBE=OFF
)

# ----------------------------
# Inner project: pmt (your lib/app)
# ----------------------------
# Ensure deps are installed first, then configure/build pmt with CMAKE_PREFIX_PATH
ExternalProject_Add(ext_pmt
  SOURCE_DIR  ${CMAKE_SOURCE_DIR}/pmt           # adjust path to your inner project
  BINARY_DIR  ${CMAKE_BINARY_DIR}/pmt-build
  DEPENDS     ext_protobuf ext_z3 ext_argparse ext_ng_log
  CMAKE_ARGS
    -DCMAKE_PREFIX_PATH=${STAGE_DIR}               # <— critical: lets find_package see staged deps
    -DCMAKE_INSTALL_PREFIX=${STAGE_DIR}            # if you also install pmt into stage
    -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
)

# Convenience target
add_custom_target(install_all
  COMMAND "${CMAKE_COMMAND}" --build . --target install
  COMMENT "Install superbuild projects into ${STAGE_DIR}"
)