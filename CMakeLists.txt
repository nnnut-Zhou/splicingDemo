cmake_minimum_required(VERSION 4.1)
project(DadeSplicing)

set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED)
find_package(OpenMVG REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories("/usr/local/include/openMVG")
include_directories(${Boost_INCLUDE_DIRS})

add_executable(DadeSplicing MyTestMain.cpp
        common.h
        DadePreprocess.cpp
        DadePreprocess.h
        DadeInterface.h
        DadeGeometry.cpp
)

target_link_libraries(DadeSplicing PRIVATE
        ${Boost_LIBRARIES}
        openMVG_exif
        openMVG_system
        openMVG_easyexif
)
