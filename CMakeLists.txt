cmake_minimum_required(VERSION 3.0)
set(CMAKE_CXX_STANDARD 11)

project(point_cloud_surface_reconstruction)

# Find Eigen3
SET(EIGEN3_INCLUDE_DIR "/usr/include/eigen3" CACHE PATH "Eigen3 include directory")
find_package(Eigen3 3.1.0 REQUIRED)

find_package(CGAL REQUIRED)
find_package(Pangolin REQUIRED)

include_directories(
  ${Pangolin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  )

create_single_source_cgal_program(main.cpp)
add_executable(${PROJECT_NAME} main.cpp Visualizer.cpp)
target_link_libraries(${PROJECT_NAME} ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} ${EIGEN3_LIBS} ${Pangolin_LIBRARIES})
