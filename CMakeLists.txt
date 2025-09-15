cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
add_compile_definitions(LV_CONF_INCLUDE_SIMPLE=1)
add_compile_definitions(LV_CONF_PATH="lv_conf.h")
include_directories(${CMAKE_SOURCE_DIR})
project(geek_recorder)
