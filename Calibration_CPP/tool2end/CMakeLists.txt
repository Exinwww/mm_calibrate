cmake_minimum_required(VERSION 2.8)
project(tool2end)

# 设置C++标准
SET(CMAKE_CXX_FLAGS "-std=c++11 -g -Wall")
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

# Get Eigen
find_package(Eigen3 REQUIRED)
include_directories("/usr/include/eigen3")

include_directories(${PROJECT_SOURCE_DIR}/include)

# 添加源文件
set(SOURCES
    app/run_tool2end.cpp
    src/tcp_tcf.cpp
)

add_executable(run_tool2end ${SOURCES})

# 链接Eigen库
target_link_libraries(run_tool2end ${EIGEN3_LIBRARIES})
