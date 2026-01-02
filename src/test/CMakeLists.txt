cmake_minimum_required(VERSION 3.10)

project(INAVTests)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(MAIN_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../..")
set(MAIN_LIB_DIR "${MAIN_DIR}/lib")
set(MAIN_SRC_DIR "${MAIN_DIR}/src/main")
set(MAIN_UTILS_DIR "${MAIN_DIR}/src/utils")
set(DOWNLOADS_DIR "${MAIN_DIR}/downloads")
set(TOOLS_DIR "${MAIN_DIR}/tools")
set(TOOLCHAIN none)

list(APPEND CMAKE_MODULE_PATH "${MAIN_DIR}/cmake")
include(main)
include(settings)

include(cmake/gtest.cmake)

enable_testing()
include(GoogleTest)
add_subdirectory(unit)
