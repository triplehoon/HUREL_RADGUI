project(fx3-bulk-test)



include_directories(${CMAKE_SOURCE_DIR}/fx3-bulk-test/include
                    ${CMAKE_SOURCE_DIR}/cruxell-communicate/include
)




add_executable(fx3-bulk-test main.cpp)

TARGET_LINK_LIBRARIES(fx3-bulk-test cyusb cruxell)

























        