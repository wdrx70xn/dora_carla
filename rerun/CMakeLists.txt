cmake_minimum_required(VERSION 3.16...3.27)

project(lidar LANGUAGES CXX)
set(CMAKE_WARN_DEVELOPER OFF)


# set(CMAKE_CXX_FLAGS_RELEASE "-O2")
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O2")
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic)


find_package(rerun_sdk REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 QUIET)

# find_package(PCL REQUIRED COMPONENTS common io features filters visualization)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


include_directories(
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  $ENV{HOME}/disk/dora_project/dora/apis/c/node/ #dora的头文件路径 node_api.h
  $ENV{HOME}/disk/dora_project/dora/apis/c/operator
  $ENV{HOME}/disk/dora_project/hongmeng/include
)

# add_executable(lidar src/test.cpp)
# # Link against rerun_sdk.
# target_link_libraries(lidar PRIVATE 
#     ${PCL_LIBRARIES}    
#     rerun_sdk
# )

add_executable(to_rerun src/points_to_rerun.cpp)
# Link against rerun_sdk.
target_link_libraries(to_rerun PRIVATE 
  ${PCL_LIBRARIES}
  $ENV{HOME}/disk/dora_project/dora/target/aarch64-unknown-linux-gnu/release/libdora_node_api_c.a
  rerun_sdk
  pthread
  m
  rt
  dl
)
