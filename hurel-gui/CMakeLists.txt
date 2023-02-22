project(hurel-gui)

# OpenGL
find_package(OpenGL REQUIRED)

# GLFW
find_package(glfw3 REQUIRED)


message(STATUS "Found GLFW3 in ${GLFW3_INCLUDE_DIR}")

include_directories(
    include
    ${CMAKE_SOURCE_DIR}/hurel-imagerecon/include
    ${CMAKE_SOURCE_DIR}/hurel-gui/include/glad
    ${CMAKE_SOURCE_DIR}/hurel-gui/include/imgui
)

file(GLOB sources src/*.cpp src/*.c src/imgui/*.cpp)
file(GLOB headers include/*.h include/glad/*.h  include/imgui/*.h)




add_executable(hurel-gui main.cpp
${sources}
${headers})



TARGET_LINK_LIBRARIES(hurel-gui 
            hurel-imagerecon                
            ${LIBRARIES}
                glfw3
                cyusb 
                nfd                                
                )

























        
