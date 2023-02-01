project(hurel-imagerecon)


if(NOT TARGET spdlog)
    # Stand-alone build
    find_package(spdlog REQUIRED)
endif()



include_directories(include)

file(GLOB sources src/*.cpp)
file(GLOB headers include/*.h)

add_library(hurelimagerecon STATIC
 ${sources}
 ${headers}
 )

TARGET_LINK_LIBRARIES(hurelimagerecon cyusb usb-1.0 ${OpenCV_LIBS})
