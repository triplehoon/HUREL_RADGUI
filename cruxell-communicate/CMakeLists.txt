project(cruxell-communicate)


if(NOT TARGET spdlog)
    # Stand-alone build
    find_package(spdlog REQUIRED)
endif()



include_directories(include)

file(GLOB sources src/*.cpp)
file(GLOB headers include/*.h)

add_library(cruxell STATIC
 ${sources}
 ${headers}
 )

 TARGET_LINK_LIBRARIES(cruxell cyusb usb-1.0)
