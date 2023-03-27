project(hurel-imagerecon)


include_directories(include
${CMAKE_SOURCE_DIR}/cruxell-communicate/include
)


file(GLOB sources src/*.cpp)
file(GLOB headers include/*.h )

add_library(hurel-imagerecon STATIC
 ${sources}
 ${headers}
 )
 message(STATUS "source: ${sources}")


TARGET_LINK_LIBRARIES(hurel-imagerecon PRIVATE
        ${LIBRARIES}
        cruxell
        )
