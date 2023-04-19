cmake_minimum_required(VERSION 3.2.2)
project(RADGUI_PROJECT)

find_package(catkin REQUIRED)


message(STATUS "rtabmap include libs: ${RTABMap_LIBRARIES}")

#set compiler as 

include_directories(./)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
SET(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# define EIGEN_USE_MKL_ALL to use MKL
add_definitions(-DEIGEN_USE_MKL_ALL)



find_package(OpenMP)

if(NOT TARGET spdlog)
    # Stand-alone build
    find_package(spdlog REQUIRED)
endif()

find_package(OpenCV 4.2  COMPONENTS core highgui imgproc imgcodecs videoio REQUIRED) # 
include_directories(${OpenCV_INCLUDE_DIRS})
message(STATUS "Opencv OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")


set(EIGEN3_INCLUDE_DIR "/usr/local/include/eigen3")
include_directories(${EIGEN3_INCLUDE_DIR})


find_package(Open3D REQUIRED)
include_directories(${OPEN3D_INCLUDE_DIRS})
include_directories("/usr/local/include/open3d/3rdparty")

find_package(RTABMap REQUIRED)
include_directories(${RTABMap_INCLUDE_DIRS})

# include python module
find_package(PythonLibs 3.8 REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

execute_process(COMMAND ${Python_EXECUTABLE} -c "import numpy; print(numpy.get_include())" OUTPUT_VARIABLE NUMPY_INCLUDE_DIR)
find_path(NUMPY_INCLUDE_DIRS numpy/arrayobject.h HINTS ${NUMPY_INCLUDE_DIR})

# find realsense2
find_package(realsense2 REQUIRED)
include_directories(${realsense2_INCLUDE_DIRS})

# find intel-mkl
find_package(MKL CONFIG REQUIRED)
include_directories(${MKL_INCLUDE_DIRS})

# fint intel-tbb
find_package(TBB CONFIG REQUIRED)
include_directories(${TBB_INCLUDE_DIRS})

# catkin_package(CATKIN_DEPENDS rtabmap)
# include_directories("/opt/ros/noetic/include/rtabmap-0.20")

find_package(PCL REQUIRED)
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
# Open3D
OpenMP::OpenMP_CXX
fmt
cyusb 
usb-1.0 
# realsense2
flann_cpp
glfw3
${OPENGL_LIBRARY}
${PYTHON_LIBRARIES}
${MKL_LIBRARIES}
TBB::tbb
)
#boost

add_subdirectory(cruxell-communicate)
add_subdirectory(fx3-bulk-test)
add_subdirectory(hurel-imagerecon)
add_subdirectory(hurel-gui)

add_dependencies(hurel-gui  hurel-imagerecon)


install(TARGETS fx3-bulk-test cruxell
         RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
         LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
 
 )   

 message(STATUS "-----------------------------------------------------")
message(STATUS "PCL include dir: ${PCL_INCLUDE_DIRS}")
message(STATUS "PCL PCL_LIBRARIES: ${PCL_LIBRARIES}")
message(STATUS "-----------------------------------------------------")
message(STATUS "open3d include dirs: ${Open3D_INCLUDE_DIRS}")
message(STATUS "open3d include libs: ${Open3D_LIBRARIES}")
message(STATUS "-----------------------------------------------------")
message(STATUS "opencv include dirs: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "opencv include libs: ${OpenCV_LIBRARIES}")
message(STATUS "-----------------------------------------------------")
message(STATUS "rtabmap include libs: ${RTABMap_LIBRARIES}")
message(STATUS "rtabmap include dir: ${RTABMap_INCLUDE_DIRS}")
message(STATUS "-----------------------------------------------------")
message(STATUS "realsense2 include dir: ${realsense2_INCLUDE_DIRS}")
message(STATUS "realsense2 include libs: ${realsense2_LIBRARIES}")
message(STATUS "-----------------------------------------------------")
message(STATUS "MKL include dir: ${MKL_INCLUDE_DIRS}")
message(STATUS "MKL include libs: ${MKL_LIBRARIES}")
message(STATUS "-----------------------------------------------------")
message(STATUS "TBB include dir: ${TBB_INCLUDE_DIRS}")
message(STATUS "TBB include libs: ${TBB_LIBRARIES}")
message(STATUS "----------------------END----------------------------")
