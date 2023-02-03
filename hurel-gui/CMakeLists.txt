project(hurel-gui)



include_directories(
    include
    ${CMAKE_SOURCE_DIR}/cruxell-communicate/include
    ${CMAKE_SOURCE_DIR}/hurel-imagerecon/include
    ${OpenCV_INCLUDE_DIRS}
)

file(GLOB sources src/*.cpp)
file(GLOB headers include/*.h)




add_executable(hurel-gui main.cpp
${sources}
${headers})



TARGET_LINK_LIBRARIES(hurel-gui 
                ${LIBRARIES}
                cyusb 
                #usb-1.0                         
                hurel-imagerecon
                )

























        