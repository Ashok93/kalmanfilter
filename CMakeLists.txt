cmake_minimum_required (VERSION 2.6)
project (kalman_filter)

add_compile_options(-std=c++11)

add_executable(main kalman_filter.h kalman_filter.cpp main.cpp)
