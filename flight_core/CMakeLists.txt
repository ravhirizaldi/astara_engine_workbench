cmake_minimum_required(VERSION 3.16)
project(astara_flight_core LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_library(astara_fsw SHARED src/astara_fsw.cpp)
target_include_directories(astara_fsw PUBLIC include)
target_compile_options(astara_fsw PRIVATE -Wall -Wextra -Wpedantic)

enable_testing()
add_executable(astara_fsw_test tests/test_fsw.cpp)
target_link_libraries(astara_fsw_test PRIVATE astara_fsw)
add_test(NAME astara_fsw_test COMMAND astara_fsw_test)
