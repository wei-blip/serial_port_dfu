cmake_minimum_required(VERSION 3.20)
set(BOARD nucleo_l152re)
#list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
#set(BOARD custom_plank)
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()
# Point to the build type and common config files
set(CONF_FILE "prj.${CMAKE_BUILD_TYPE}.conf" "prj.conf")
# CMake sets some default compiler flags in CMAKE_<LANG>_FLAGS_<CONFIG>. Zephyr sets all required flags from KConfig so
# clear CMake defaults to avoid any conflicts.
foreach(lang C CXX)
    set(CMAKE_${lang}_FLAGS "")
    foreach(type DEBUG RELEASE RELWITHDEBINFO MINSIZEREL)
        set(CMAKE_${lang}_FLAGS_${type} "")
    endforeach()
endforeach()
# Suppress a warning from Zephyr about a mismatch between CMAKE_<LANG>_FLAGS_<CONFIG> optimization flags (now empty) and
# Zephyr's KConfig optimization flags.
set(NO_BUILD_TYPE_WARNING ON)

find_package(Zephyr  2.7.0 EXACT)
project(serial_dfu)
#project(app LANGUAGES C VERSION 1.0.0)

FILE(GLOB_RECURSE SOURCES src/*.*)

target_sources(app PRIVATE ${SOURCES})
#target_include_directories(app PRIVATE /home/rts/zephyrproject/modules/lib)
