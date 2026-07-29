cmake_minimum_required(VERSION 3.16)
project(fsw_core LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(FSW_INTERNAL_SOURCES
    src/control/control.cpp
    src/core/controller.cpp
    src/faults/fault_manager.cpp
    src/guidance/guidance.cpp
    src/mission/commands.cpp
    src/mission/mission.cpp
    src/navigation/attitude.cpp
    src/navigation/navigation.cpp
    src/output/output_builder.cpp
    src/sensors/barometer_voting.cpp
    src/sensors/gnss_voting.cpp
    src/sensors/imu_voting.cpp
    src/sensors/magnetometer_voting.cpp
    src/sensors/sensor_voting.cpp
    src/validation/config_validation.cpp
    src/validation/input_validation.cpp
)

add_library(fsw_core_internal STATIC ${FSW_INTERNAL_SOURCES})
target_include_directories(
    fsw_core_internal
    PRIVATE
        include
        src
)
target_compile_options(
    fsw_core_internal
    PRIVATE
        -Wall
        -Wextra
        -Wpedantic
)
set_target_properties(
    fsw_core_internal
    PROPERTIES
        CXX_VISIBILITY_PRESET hidden
        VISIBILITY_INLINES_HIDDEN YES
)

add_library(fsw_core SHARED src/api/fsw_c_api.cpp)
target_link_libraries(fsw_core PRIVATE fsw_core_internal)
target_include_directories(
    fsw_core
    PUBLIC include
    PRIVATE src
)
target_compile_options(fsw_core PRIVATE -Wall -Wextra -Wpedantic)
set_target_properties(
    fsw_core
    PROPERTIES
        CXX_VISIBILITY_PRESET hidden
        VISIBILITY_INLINES_HIDDEN YES
)

enable_testing()

function(add_fsw_unit_test name source)
    add_executable(${name} ${source})
    target_link_libraries(${name} PRIVATE fsw_core_internal fsw_core)
    target_include_directories(${name} PRIVATE include src tests)
    add_test(NAME ${name} COMMAND ${name})
endfunction()

add_fsw_unit_test(fsw_validation_test tests/unit/test_validation.cpp)
add_fsw_unit_test(fsw_sensor_voting_test tests/unit/test_sensor_voting.cpp)
add_fsw_unit_test(fsw_navigation_test tests/unit/test_navigation.cpp)
add_fsw_unit_test(fsw_attitude_test tests/unit/test_attitude.cpp)
add_fsw_unit_test(fsw_mission_test tests/unit/test_mission.cpp)
add_fsw_unit_test(fsw_faults_test tests/unit/test_faults.cpp)
add_fsw_unit_test(fsw_control_test tests/unit/test_control.cpp)

add_executable(fsw_api_test tests/integration/test_fsw_api.cpp)
target_link_libraries(fsw_api_test PRIVATE fsw_core)
target_include_directories(fsw_api_test PRIVATE include tests)
add_test(NAME fsw_api_test COMMAND fsw_api_test)
