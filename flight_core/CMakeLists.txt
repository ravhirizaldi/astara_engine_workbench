cmake_minimum_required(VERSION 3.16)
project(fsw_core LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_library(fsw_core SHARED src/fsw.cpp)
target_include_directories(fsw_core PUBLIC include)
target_compile_options(fsw_core PRIVATE -Wall -Wextra -Wpedantic)

enable_testing()
add_executable(fsw_core_test tests/test_fsw.cpp)
target_link_libraries(fsw_core_test PRIVATE fsw_core)
add_test(NAME fsw_core_test COMMAND fsw_core_test)
