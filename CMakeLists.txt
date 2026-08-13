cmake_minimum_required(VERSION 3.5)

project(UGV-Coordination-System)

find_package(Boost 1.58 REQUIRED COMPONENTS program_options)
find_package(PkgConfig)
pkg_check_modules(YamlCpp yaml-cpp)

## OPEN MOTION PLANNING LIBRARY (Reeds-Shepp state space, reused from CL-CBS)
## Config-mode find_package(OMPL) is fragile across distros (the exported
## config file's case varies: OMPLConfig.cmake vs omplConfig.cmake). Prefer
## pkg-config, which every OMPL packaging ships consistently, and fall back
## to find_package(ompl) for systems built from source with the upstream
## default (lowercase) package name.
find_package(PkgConfig REQUIRED)
pkg_check_modules(OMPL ompl)
if(NOT OMPL_FOUND)
  find_package(ompl QUIET)
  if(ompl_FOUND)
    set(OMPL_FOUND TRUE)
    set(OMPL_LIBRARIES ompl)
  endif()
endif()
if(NOT OMPL_FOUND)
  message(FATAL_ERROR "Open Motion Planning Library (OMPL) not found via pkg-config or find_package(ompl).")
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if (${CMAKE_BUILD_TYPE} MATCHES "Release")
  message("Release mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -O3")
else(${CMAKE_BUILD_TYPE} MATCHES "Release")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
endif (${CMAKE_BUILD_TYPE} MATCHES "Release")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include_directories(
  include
  ${OMPL_INCLUDE_DIRS}
)

## Main pipeline: Stage 1 (nominal generation) + Stage 2 (Hungarian
## allocation) + Stage 3 (CL-CBS baseline or LCBR conflict resolution).
add_executable(ugv_coordination
  src/ugv_coordination.cpp
)
target_link_libraries(ugv_coordination
  ${Boost_LIBRARIES}
  ${OMPL_LIBRARIES}
  yaml-cpp
)

# clang-format (kept for consistency with the original CL-CBS tooling)
set(ALL_SOURCE_FILES
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
add_custom_target(clang-format
  COMMAND clang-format -style=Google -i ${ALL_SOURCE_FILES}
)

# Tests (tests/CMakeLists.txt) -- `ctest` after building, or `ctest -V`
# for verbose output. See tests/CMakeLists.txt for what is and isn't
# wired in yet.
enable_testing()
add_subdirectory(tests)

