cmake_minimum_required(VERSION 2.8)
project(lidar2car)

# compile option
SET(CMAKE_CXX_FLAGS "-std=c++11 -g -Wall") 

# compile output path 
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

# 查找Python库
find_package(PythonLibs 3.8 REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

# Get boost
find_package(Boost REQUIRED filesystem)
include_directories(${BOOST_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

# Get Eigen
find_package(Eigen3 REQUIRED)
include_directories("/usr/include/eigen3")

# Get ceres
add_subdirectory(ceres)
include_directories(${PROJECT_SOURCE_DIR}/ceres/include)

# Get PCL
set(PCL_DIR "/usr/lib/x86_64-linux-gnu/cmake/pcl/")
find_package(PCL 1.10 REQUIRED 
             PATHS "/usr/local/pcl-1.10/share/pcl-1.10/" NO_DEFAULT_PATH)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Get OpenCV
set(OpenCV_DIR "/usr/lib/x86_64-linux-gnu/cmake/opencv4/")
find_package(OpenCV REQUIRED 
             PATHS "/usr/local/opencv-2.4.13/share/OpenCV/" NO_DEFAULT_PATH)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_INSTALL_PATH}/lib) 

# Get splinter
include_directories(${PROJECT_SOURCE_DIR}/3rdparty/splinter-3.0/include)
link_directories(${PROJECT_SOURCE_DIR}/lib)

# Get matplotlib
#include_directories("/usr/include/python2.7")
include_directories(${PROJECT_SOURCE_DIR}/3rdparty/matplotlib-cpp)

# other dependence
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/include/fastloam)
include_directories(${PROJECT_SOURCE_DIR}/include/yawcalib)
include_directories(${PROJECT_SOURCE_DIR}/include/rpzcalib)

SET(DEP_LIBS
    ${Boost_LIBRARIES} 
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    splinter-static-3-0
    # splinter-3-0
    ${PYTHON_LIBRARIES} # 修改为Python 3.8
    ceres
)

file(GLOB_RECURSE SRC_LIST src/*.cpp)

add_library(${PROJECT_NAME} STATIC ${SRC_LIST})

TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${DEP_LIBS} -lgomp)

ADD_EXECUTABLE(run_prh ${PROJECT_SOURCE_DIR}/app/run_prh.cpp)
TARGET_LINK_LIBRARIES(run_prh ${PROJECT_NAME} ${PYTHON_LIBRARIES}) # 确保链接到正确的 Python 库

ADD_EXECUTABLE(run_yaw ${PROJECT_SOURCE_DIR}/app/run_yaw.cpp)
TARGET_LINK_LIBRARIES(run_yaw ${PROJECT_NAME} ${PYTHON_LIBRARIES}) # 确保链接到正确的 Python 库