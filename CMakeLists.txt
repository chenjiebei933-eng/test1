cmake_minimum_required(VERSION 3.16)

set(IDF_TARGET "esp32c2" CACHE STRING "Target chip")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp32c2_qmi8658a)
