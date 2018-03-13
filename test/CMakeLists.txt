set(TEST_NAME "Test")
set(BM_NAME "BM")

set(sources
  main.cpp
  test_compose.cpp
  test_io.cpp
  test_protocol.cpp
  test_coordinates.cpp
  test_map.cpp
  test_lane.cpp
  test_point.cpp
  test_path.cpp
  )

set(benchmark
  bm_frenet.cpp
)

include_directories(${CMAKE_SOURCE_DIR}/src)

add_executable(${TEST_NAME} ${sources} $<TARGET_OBJECTS:lib${CMAKE_PROJECT_NAME}>)
add_dependencies(${TEST_NAME} lib${CMAKE_PROJECT_NAME})

add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})

if(CMAKE_BUILD_TYPE STREQUAL "Coverage")
  include(EnableCoverage)
endif()

#link_directories(~/Documents/Projects/benchmark/build/src)
#include_directories(~/Documents/Projects/benchmark/include)

#add_executable(${BM_NAME} ${benchmark} $<TARGET_OBJECTS:lib${CMAKE_PROJECT_NAME}>)
#add_dependencies(${BM_NAME} lib${CMAKE_PROJECT_NAME})
#target_link_libraries(${BM_NAME} benchmark)
