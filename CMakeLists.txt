cmake_minimum_required(VERSION 3.8)
set(CMAKE_CXX_STANDARD 14)
project(MyntEyeVision)

# cmake modules
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)


###############################################################################
# Libraries
###############################################################################
find_package(Gflags REQUIRED)                   # gflags
find_package(Glog REQUIRED)                     # glog
find_package(Boost COMPONENTS filesystem REQUIRED)    # boost
find_package(OpenCV REQUIRED)                   # OpenCV
find_package(mynteyed REQUIRED)                 # MyntEye
# prive dependency include directories and libraries
list(APPEND DEPEND_INCLUDES
    ${GFLAGS_INCLUDE_DIRS}
    ${GLOG_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    )
list(APPEND DEPEND_LIBS
    ${GFLAGS_LIBRARIES}
    ${GLOG_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
    mynteye_depth
    )

# when SDK build with OpenCV, add WITH_OPENCV macro to enable some features depending on OpenCV, such as ToMat().
if (mynteyed_WITH_OPENCV)
    add_definitions(-DWITH_OPENCV)
endif ()

###############################################################################
# Project Setting
###############################################################################
# default build type
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif ()
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")

# fmt: header only library
add_definitions(-DFMT_HEADER_ONLY)

# export compile commands
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# projects build setting
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)



###############################################################################
# Project
###############################################################################
# third party
set(THIRD_PATH ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty)
list(APPEND DEPEND_INCLUDES
    ${THIRD_PATH}
    ${THIRD_PATH}/cxxopts
    )

# the main project
add_executable(${PROJECT_NAME} main.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE ${DEPEND_INCLUDES})
target_link_libraries(${PROJECT_NAME} PRIVATE ${DEPEND_LIBS})

# data recorder
add_executable(recorder recorder.cpp)
target_include_directories(recorder PRIVATE ${DEPEND_INCLUDES})
target_link_libraries(recorder PRIVATE ${DEPEND_LIBS})