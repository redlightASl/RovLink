cmake_minimum_required(VERSION 3.5)
project(crc_tb)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

file(GLOB CUR_SRCS "*.c" "*.cpp")
add_executable(${PROJECT_NAME} ${CUR_SRCS})
