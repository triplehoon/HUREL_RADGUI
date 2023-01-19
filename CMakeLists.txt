cmake_minimum_required(VERSION 3.2.2)
project(RADGUI_PROJECT)

add_subdirectory(fx3-bulk-test)


install(TARGETS fx3-bulk-test.o
        RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
)   