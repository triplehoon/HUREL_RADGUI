add_executable(fx3-bulk-test.o main.cpp)

include_directories(${CMAKE_SOURCE_DIR}/fx3-bulk-test)

TARGET_LINK_LIBRARIES(fx3-bulk-test.o cyusb)
























        