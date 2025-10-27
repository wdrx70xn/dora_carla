cmake_minimum_required(VERSION 3.10)
project(cmd_trans_node)

# 设置默认构建类型
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# 设置编译器标志
set(CMAKE_CXX_FLAGS "-std=c++14 -pthread -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

# 设置当前目录
set(CURRENT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
message(STATUS "Current dir stored in variable: ${CURRENT_DIR}")

# 查找所需的库
find_library(LIBRT rt)
find_library(LIBDL dl)
find_library(LIBPTHREAD pthread)
find_library(LIBM m)

# 检查库是否找到
if(NOT LIBRT)
    message(FATAL_ERROR "Library rt not found")
endif()
if(NOT LIBDL)
    message(FATAL_ERROR "Library dl not found")
endif()
if(NOT LIBPTHREAD)
    message(FATAL_ERROR "Library pthread not found")
endif()
if(NOT LIBM)
    message(FATAL_ERROR "Library m not found")
endif()

# 设置包含目录
include_directories(
  include
  # 假设 nlohmann/json 库在项目根目录下的 nlohmann 文件夹中
  ${CURRENT_DIR}/nlohmann  
  $ENV{HOME}/disk/dora_project/dora/apis/c/node/ #dora的头文件路径 node_api.h
  $ENV{HOME}/disk/dora_project/dora/apis/c/operator
)

# 添加可执行文件

add_executable(ctrl_trans src/control_cmd_trans.cpp)


target_link_libraries(ctrl_trans
$ENV{HOME}/disk/dora_project/dora/target/aarch64-unknown-linux-gnu/release/libdora_node_api_c.a
${LIBM}
  ${LIBRT}
  ${LIBDL}
  ${LIBPTHREAD}
)
