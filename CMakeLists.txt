cmake_minimum_required(VERSION 3.2.2)
project(RADGUI_PROJECT)



include_directories(./)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -mavx2")
SET(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

find_package(OpenMP)


if(NOT TARGET spdlog)
    # Stand-alone build
    find_package(spdlog REQUIRED)
endif()

find_package(OpenCV 4 COMPONENTS core highgui imgproc imgcodecs REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
message(STATUS "Opencv OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")


set(EIGEN3_INCLUDE_DIR "/usr/local/include/eigen3")
include_directories(${EIGEN3_INCLUDE_DIR})


find_package(Open3D REQUIRED)
include_directories(${OPEN3D_INCLUDE_DIRS})
include_directories("/usr/local/include/open3d/3rdparty")


find_package(RTABMap REQUIRED)
include_directories("/usr/local/include/rtabmap-0.20")

find_package(PCL 1.13 REQUIRED)

message(STATUS "PCL PCL_LIBRARIES: ${PCL_LIBRARIES}")
message(STATUS "PCL PCL_DIRS: ${Open3D_INCLUDE_DIRS}")

include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})



set(LIBRARIES
${RTABMap_LIBRARIES}
${PCL_LIBRARIES}
${OpenCV_LIBS}
${Open3D_LIBS}
tbb
Open3D
OpenMP::OpenMP_CXX
fmt
cyusb 
usb-1.0 
realsense2
flann_cpp
#boost
)

add_subdirectory(cruxell-communicate)
add_subdirectory(fx3-bulk-test)
add_subdirectory(hurel-imagerecon)
add_subdirectory(hurel-gui)


install(TARGETS fx3-bulk-test cruxell
         RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
         LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
 
 )   
