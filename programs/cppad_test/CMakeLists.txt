cmake_minimum_required(VERSION 2.8.3)
project(cppad_test)

add_compile_options(-std=c++11)

find_package(PkgConfig REQUIRED)
pkg_check_modules(CPPAD cppad REQUIRED)
include_directories(${CPPAD_INCLUDEDIR})

add_executable(cppad_test main.cc)
