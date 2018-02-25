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
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

find_package(Threads REQUIRED)
find_package(ZLIB REQUIRED)

include(FindPkgConfig)
# if(PKG_CONFIG_EXECUTABLE)
# 	set(PKG_CONFIG_PATH "/usr/local/lib/pkgconfig:/usr/share/pkgconfig:${PKG_CONFIG_PATH}")
# 	pkg_search_module(CPPAD REQUIRED cppad)
# 	pkg_search_module(IPOPT REQUIRED ipopt>=3.12.7)
# 	include_directories(${IPOPT_INCLUDEDIR}/..)
# else()
# 	message(STATUS "PKG_CONFIG_EXECUTABLE not set. Trying find required modules manually.")
# 	find_path(CPPAD_INCLUDE_DIR NAMES cppad/cppad.hpp)
# 	find_package_handle_standard_args(CPPAD DEFAULT_MSG CPPAD_INCLUDE_DIR)

# 	find_path(IPOPT_INCLUDE_DIR NAME coin/IpoptConfig.h PATHS /usr/local/include)
# 	find_library(IPOPT_LIBRARIES ipopt)
# 	find_package_handle_standard_args(IPOPT DEFAULT_MSG IPOPT_INCLUDE_DIR IPOPT_LIBRARIES)
# 	include_directories(${IPOPT_INCLUDE_DIR})
# 	get_filename_component(IPOPT_LIBDIR ${IPOPT_LIBRARIES} DIRECTORY)
# endif()
# link_directories(${IPOPT_LIBDIR})

find_library(UWS_LIBRARIES uWS)
find_package_handle_standard_args(UWS DEFAULT_MSG UWS_LIBRARIES)

include_directories(thirdparty)
include_directories(thirdparty/Eigen-3.3)

include(CTest)
enable_testing()
add_subdirectory(src)
add_subdirectory(test)
