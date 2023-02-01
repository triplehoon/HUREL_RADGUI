cmake_minimum_required(VERSION 3.2.2)
project(RADGUI_PROJECT)



include_directories(./)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")


if(NOT TARGET spdlog)
    # Stand-alone build
    find_package(spdlog REQUIRED)
endif()
find_package(OpenCV REQUIRED)
include_directories(${Opencv_INCLUDE_DIRS})


find_package(Eigen3 REQUIRED)
find_package(Open3D REQUIRED)




add_subdirectory(cruxell-communicate)
add_subdirectory(fx3-bulk-test)
add_subdirectory(hurel-imagerecon)

 install(TARGETS fx3-bulk-test cruxell
         RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
         LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
 
 )   
