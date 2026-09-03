cmake_minimum_required(VERSION 3.15)

project(UGV-Coordination-System LANGUAGES C CXX)

# ============================================================
# C++ configuration
# ============================================================

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_BUILD_TYPE MATCHES "Release")
    message(STATUS "Release mode")
    add_compile_options(-Wall -Wextra -O3)
else()
    add_compile_options(-Wall -Wextra)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ============================================================
# Boost
# ============================================================

find_package(
    Boost 1.58
    REQUIRED
    COMPONENTS program_options
)

message(STATUS "Boost version: ${Boost_VERSION}")
message(STATUS "Boost include dirs: ${Boost_INCLUDE_DIRS}")

# ============================================================
# yaml-cpp
# ============================================================

find_package(PkgConfig REQUIRED)

pkg_check_modules(
    YamlCpp
    REQUIRED
    yaml-cpp
)

message(STATUS "yaml-cpp version: ${YamlCpp_VERSION}")

# ============================================================
# OMPL
# ============================================================

pkg_check_modules(OMPL QUIET ompl)

if(NOT OMPL_FOUND)

    find_package(
        ompl
        CONFIG
        QUIET
    )

    if(ompl_FOUND)

        set(OMPL_FOUND TRUE)

        if(NOT OMPL_LIBRARIES)
            set(OMPL_LIBRARIES ompl)
        endif()

    endif()

endif()

if(NOT OMPL_FOUND)

    find_path(
        OMPL_LOCAL_INCLUDE_DIR
        NAMES
            ompl/base/StateSpace.h
        HINTS
            "$ENV{HOME}/.local/include/ompl-1.7"
    )

    find_library(
        OMPL_LOCAL_LIBRARY
        NAMES
            ompl
        HINTS
            "$ENV{HOME}/.local/lib64"
            "$ENV{HOME}/.local/lib"
    )

    if(
        OMPL_LOCAL_INCLUDE_DIR
        AND
        OMPL_LOCAL_LIBRARY
    )

        message(
            STATUS
            "Using local OMPL installation"
        )

        set(
            OMPL_INCLUDE_DIRS
            "${OMPL_LOCAL_INCLUDE_DIR}"
        )

        set(
            OMPL_LIBRARIES
            "${OMPL_LOCAL_LIBRARY}"
        )

        set(
            OMPL_FOUND TRUE
        )

    endif()

endif()

if(NOT OMPL_FOUND)

    message(
        FATAL_ERROR
        "Open Motion Planning Library (OMPL) not found."
    )

endif()

message(
    STATUS
    "OMPL include dirs: ${OMPL_INCLUDE_DIRS}"
)

message(
    STATUS
    "OMPL libraries: ${OMPL_LIBRARIES}"
)

# ============================================================
# Main executable
# ============================================================

add_executable(
    ugv_coordination
    src/ugv_coordination.cpp
)

target_include_directories(
    ugv_coordination
    PRIVATE

        ${CMAKE_CURRENT_SOURCE_DIR}/include

        ${Boost_INCLUDE_DIRS}

        ${YamlCpp_INCLUDE_DIRS}

        ${OMPL_INCLUDE_DIRS}
)

target_link_directories(
    ugv_coordination
    PRIVATE

        ${YamlCpp_LIBRARY_DIRS}

        $ENV{HOME}/.local/lib64
        $ENV{HOME}/.local/lib
)

target_link_libraries(
    ugv_coordination
    PRIVATE

        Boost::program_options

        ${OMPL_LIBRARIES}

        ${YamlCpp_LIBRARIES}
)

# ============================================================
# Runtime search path for locally installed OMPL
# ============================================================

set_target_properties(
    ugv_coordination
    PROPERTIES

        BUILD_RPATH
        "$ENV{HOME}/.local/lib64;$ENV{HOME}/.local/lib"

        INSTALL_RPATH
        "$ENV{HOME}/.local/lib64;$ENV{HOME}/.local/lib"
)

# ============================================================
# clang-format
# ============================================================

set(
    ALL_SOURCE_FILES

    ${CMAKE_CURRENT_SOURCE_DIR}/include/hybrid_astar.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/environment.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/neighbor.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/planresult.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/timer.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/task_allocation.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/local_repair.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/body_conflict_tree.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/low_level_environment.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/experiment_logger.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/ugv_coordination.cpp
)

add_custom_target(
    clang-format

    COMMAND
        clang-format
        -style=Google
        -i
        ${ALL_SOURCE_FILES}
)

# ============================================================
# Tests
# ============================================================

enable_testing()

add_subdirectory(tests)
