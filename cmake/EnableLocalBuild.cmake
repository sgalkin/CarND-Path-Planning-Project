cmake_minimum_required (VERSION 3.5)

enable_language(CXX)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_VERBOSE_MAKEFILE OFF)

if(CMAKE_COMPILER_IS_GNUCXX AND
   CMAKE_CXX_COMPILER_VERSION VERSION_LESS 5.4)
  message(FATAL_ERROR "GCC version must be at least 5.4!")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Werror")

find_package(Threads REQUIRED)
find_package(ZLIB REQUIRED)

include(FindPkgConfig)

find_library(UWS_LIBRARIES uWS)
find_package_handle_standard_args(UWS DEFAULT_MSG UWS_LIBRARIES)

include_directories(thirdparty)
include_directories(thirdparty/Eigen-3.3)

include(CTest)
enable_testing()
add_subdirectory(src)
add_subdirectory(test)

add_custom_target(
  LinkData ALL
  COMMAND 
    ${CMAKE_COMMAND} -E 
    copy_directory "${CMAKE_SOURCE_DIR}/data" "${CMAKE_CURRENT_BINARY_DIR}/data"
)
