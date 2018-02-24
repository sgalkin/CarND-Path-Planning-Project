find_program(GCOV_PATH gcov)

if(NOT GCOV_PATH)
	message(FATAL_ERROR "gcov not found!")
endif()

if("${CMAKE_CXX_COMPILER_ID}" MATCHES "(Apple)?[Cc]lang")
	if("${CMAKE_CXX_COMPILER_VERSION}" VERSION_LESS 3)
		message(FATAL_ERROR "Clang version must be 3.0.0 or greater")
	endif()
elseif(NOT CMAKE_COMPILER_IS_GNUCXX)
	message(FATAL_ERROR "Unsupported compiler ${CMAKE_CXX_COMPILER_ID}")
endif()

set(CMAKE_CXX_FLAGS_COVERAGE
    "-g -O0 --coverage -fprofile-arcs -ftest-coverage"
    CACHE STRING "coverage build C++ compiler flags"
    FORCE
)
set(CMAKE_C_FLAGS_COVERAGE
    "-g -O0 --coverage -fprofile-arcs -ftest-coverage"
    CACHE STRING "coverage build C compiler flags"
    FORCE
)
  
mark_as_advanced(
    CMAKE_CXX_FLAGS_COVERAGE
    CMAKE_C_FLAGS_COVERAGE
)
