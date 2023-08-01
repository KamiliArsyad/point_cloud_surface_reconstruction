cmake_minimum_required(VERSION 3.0)
set(CMAKE_CXX_STANDARD 11)

project(point_cloud_surface_reconstruction)

find_package(CGAL REQUIRED)

create_single_source_cgal_program(main.cpp)
