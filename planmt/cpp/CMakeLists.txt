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
# External: spdlog (header)
# ----------------------------
ExternalProject_Add(ext_spdlog
  GIT_REPOSITORY https://github.com/gabime/spdlog.git
  GIT_TAG        v1.9.2
  GIT_SHALLOW    TRUE
  UPDATE_DISCONNECTED TRUE
  CMAKE_ARGS
    ${COMMON_ARGS}
    -DSPDLOG_BUILD_TESTS=OFF
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
# External: corrade
# ----------------------------
ExternalProject_Add(ext_corrade
  GIT_REPOSITORY https://github.com/mosra/corrade.git
  GIT_TAG        5f66f3c
  GIT_SHALLOW    TRUE
  GIT_SUBMODULES ""
  UPDATE_DISCONNECTED TRUE
  CMAKE_ARGS
    ${COMMON_ARGS}
    # -DWITH_INTERCONNECT=OFF
    -DCORRADE_WITH_TESTSUITE=OFF
    # -DBUILD_DEPRECATED=OFF
    # -DWITH_RC=OFF
    # -DWITH_MAIN=OFF
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5
)

# ----------------------------
# Inner project: pmt (your lib/app)
# ----------------------------
# Ensure deps are installed first, then configure/build pmt with CMAKE_PREFIX_PATH
ExternalProject_Add(ext_pmt
  SOURCE_DIR  ${CMAKE_SOURCE_DIR}/pmt           # adjust path to your inner project
  BINARY_DIR  ${CMAKE_BINARY_DIR}/pmt-build
  DEPENDS     ext_protobuf ext_z3 ext_argparse ext_spdlog ext_corrade
  BUILD_ALWAYS TRUE
  CMAKE_ARGS
    -DCMAKE_PREFIX_PATH=${STAGE_DIR}               # <— critical: lets find_package see staged deps
    -DCMAKE_INSTALL_PREFIX=${STAGE_DIR}            # if you also install pmt into stage
    -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
    -DCMAKE_SKIP_INSTALL_RPATH=ON                  # Skip RPATH manipulation during install
)

# Convenience target
add_custom_target(install_all
  COMMAND "${CMAKE_COMMAND}" --build . --target install
  COMMENT "Install superbuild projects into ${STAGE_DIR}"
)