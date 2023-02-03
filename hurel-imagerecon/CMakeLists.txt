project(hurel-imagerecon)


include_directories(include {OpenCV_INCLUDE_DIRS})


file(GLOB sources src/*.cpp)
file(GLOB headers include/*.h)

add_library(hurel-imagerecon STATIC
 ${sources}
 ${headers}
 )


TARGET_LINK_LIBRARIES(hurel-imagerecon PUBLIC
        ${LIBRARIES}       
         )
