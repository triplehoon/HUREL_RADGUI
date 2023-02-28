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

find_package(OpenCV 4  COMPONENTS core highgui imgproc imgcodecs videoio REQUIRED) # 
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
message(STATUS "open3d include dirs: ${Open3D_INCLUDE_DIRS}")
message(STATUS "open3d include libs: ${Open3D_LIBRARIES}")

message(STATUS "opencv include libs: ${OpenCV_LIBRARIES}")

include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})


# GLFW
find_package(glfw3 REQUIRED)

find_package(OpenGL REQUIRED)

set(LIBRARIES
${RTABMap_LIBRARIES}
${PCL_LIBRARIES}
${OpenCV_LIBRARIES}
${Open3D_LIBRARIES}
tbb
# Open3D
OpenMP::OpenMP_CXX
fmt
cyusb 
usb-1.0 
realsense2
flann_cpp
glfw3
${OPENGL_LIBRARY}
#boost
)

add_subdirectory(cruxell-communicate)
add_subdirectory(fx3-bulk-test)
add_subdirectory(hurel-imagerecon)
add_subdirectory(hurel-gui)

add_dependencies(hurel-gui  hurel-imagerecon)


install(TARGETS fx3-bulk-test cruxell
         RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
         LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
 
 )   
