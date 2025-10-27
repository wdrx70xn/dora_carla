cmake_minimum_required(VERSION 2.8.3)
# set compiler
set(CMAKE_CXX_COMPILER "/usr/bin/clang++")

# 合并项目名称，这里可以根据需求修改
project(CombinedProject)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-std=c++14 -pthread -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
# NOTE: CHOOSE THE O2/O3 is really important for speed! check more on optimization on compile
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")

set(CURRENT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
message(STATUS "Current dir stored in variable: ${CURRENT_DIR}")

# 查找依赖项
find_package(PCL REQUIRED)
find_package(Eigen3 QUIET)
find_package(PkgConfig)

pkg_check_modules(YAMLCPP REQUIRED yaml-cpp>=0.5)

# nlohmann/json
find_path(NLOHMANN_JSON_INCLUDE_DIR nlohmann/json.hpp PATHS ${CMAKE_CURRENT_SOURCE_DIR}/include /usr/local/include)
if(NOT NLOHMANN_JSON_INCLUDE_DIR)
  message(FATAL_ERROR "nlohmann/json.hpp not found. Please install it or specify the include path.")
endif()

if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

message(${EIGEN_INCLUDE_DIRS})

# 合并包含目录
include_directories(
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${YAMLCPP_INCLUDE_DIRS}
  ${NLOHMANN_JSON_INCLUDE_DIR}
  $ENV{HOME}/disk/dora_project/dora/apis/c/node #dora的头文件路径 node_api.h
  $ENV{HOME}/disk/dora_project/dora/apis/c/operator
  ${CURRENT_DIR}/rs_driver/src #雷达的头文件路径
  $ENV{HOME}/disk/dora_project/hongmeng/include
  ${CURRENT_DIR}/include
  $ENV{HOME}/disk/dora_project/hongmeng/dora-hardware/vendors/gnss/CGI_610/include
  $ENV{HOME}/disk/dora_project/hongmeng/dora-hardware/vendors/carla_bridge/liadr_receive/rs_driver/utility # 添加 sync_queue.hpp 所在目录
  #$ENV{HOME}/disk/dora_project/hongmeng/dora-hardware/vendors/carla_bridge/liadr_receive/rs_driver/common
)

# 添加可执行文件
add_executable(one one.cpp)


# 链接库
target_link_libraries(one
  ${PCL_LIBRARIES}
  ${EIGEN_LIBRARIES}
  ${YAMLCPP_LIBRARIES}
  $ENV{HOME}/disk/dora_project/dora/target/aarch64-unknown-linux-gnu/release/libdora_node_api_c.a
  m
  rt
  dl 
  pthread
  pcap
)

   
