cmake_minimum_required(VERSION 2.8)
project(handEye)

# 设置C++标准
SET(CMAKE_CXX_FLAGS "-std=c++11 -g -Wall")
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

# Get Eigen
find_package(Eigen3 REQUIRED)
include_directories("/usr/include/eigen3")

#Get OpenCV
set(OpenCV_DIR "/usr/lib/x86_64-linux-gnu/cmake/opencv4/")
find_package(OpenCV REQUIRED 
             PATHS "/usr/local/opencv-2.4.13/share/OpenCV/" NO_DEFAULT_PATH)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_INSTALL_PATH}/lib) 

include_directories(${PROJECT_SOURCE_DIR}/include)

# 添加源文件
set(SOURCES
    app/run_handEye.cpp
    src/handEye.cpp
)
SET(DEP_LIBS
    ${EIGEN3_LIBRARIES}
    ${OpenCV_LIBS}
)

add_executable(run_handEye ${SOURCES})

TARGET_LINK_LIBRARIES(run_handEye ${DEP_LIBS} -lgomp)
