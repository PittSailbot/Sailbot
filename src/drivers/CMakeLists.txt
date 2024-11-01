cmake_minimum_required(VERSION 3.15...3.27)
project(drivers)

if (CMAKE_VERSION VERSION_LESS 3.18)
  set(DEV_MODULE Development)
else()
  set(DEV_MODULE Development.Module)
endif()

find_package(Python 3.10 COMPONENTS Interpreter ${DEV_MODULE} REQUIRED)

if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

# Detect the installed nanobind package and import it into CMake
execute_process(
  COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
  OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE nanobind_ROOT)
find_package(nanobind CONFIG REQUIRED)

# Install dependencies
INCLUDES = -I/path/to/arduino/includes -I/path/to/other/includes
LIBRARY_PATHS = -L/path/to/arduino/libraries -L/path/to/other/libraries
LIBRARIES = -larduino -lsomeotherlib

# TODO: add lib dependencies from pio
# find_package(Arduino REQUIRED)
find_package(Adafruit_GPS)

# TODO: Migrate when Teensy is deprecated
nanobind_add_module(drivers src/gps.cpp)
