cmake_minimum_required(VERSION 3.2.2)
project(RADGUI_PROJECT)



include_directories(./)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")


if(NOT TARGET spdlog)
    # Stand-alone build
    find_package(spdlog REQUIRED)
endif()





add_subdirectory(cruxell-communicate)
add_subdirectory(fx3-bulk-test)


 install(TARGETS fx3-bulk-test cruxell
         RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
         LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
 
 )   
