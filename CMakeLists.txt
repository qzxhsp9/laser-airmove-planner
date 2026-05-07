cmake_minimum_required(VERSION 3.20)
project(laser_airmove_planner VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(AIRMOVE_BUILD_EXAMPLES "Build examples" ON)
option(AIRMOVE_BUILD_TOOLS "Build command line tools" ON)
option(AIRMOVE_BUILD_TESTS "Build tests" OFF)

find_package(ompl REQUIRED)
find_package(fcl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)
find_package(ruckig REQUIRED)

add_library(airmove_planner
    src/AirMovePlanner.cpp
    src/CollisionWorld.cpp
    src/BSpline.cpp
    src/RuckigExecutor.cpp
    src/StlMeshLoader.cpp
    src/JsonIO.cpp
)

add_library(airmove::planner ALIAS airmove_planner)

target_include_directories(airmove_planner
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(airmove_planner
    PUBLIC
        ompl::ompl
        Eigen3::Eigen
        nlohmann_json::nlohmann_json
        ruckig::ruckig
        fcl
)

target_compile_options(airmove_planner PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra -Wpedantic>
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
)

if(AIRMOVE_BUILD_EXAMPLES)
    add_executable(single_airmove_demo examples/single_airmove_demo.cpp)
    target_link_libraries(single_airmove_demo PRIVATE airmove::planner)
endif()

if(AIRMOVE_BUILD_TOOLS)
    add_executable(laser_airmove_plan tools/laser_airmove_plan.cpp)
    target_link_libraries(laser_airmove_plan PRIVATE airmove::planner)

    add_executable(laser_airmove_benchmark tools/laser_airmove_benchmark.cpp)
    target_link_libraries(laser_airmove_benchmark PRIVATE airmove::planner)
endif()

if(AIRMOVE_BUILD_TESTS)
    enable_testing()
    add_executable(airmove_basic_tests tests/basic_tests.cpp)
    target_link_libraries(airmove_basic_tests PRIVATE airmove::planner)
    add_test(NAME airmove_basic_tests COMMAND airmove_basic_tests)
endif()
