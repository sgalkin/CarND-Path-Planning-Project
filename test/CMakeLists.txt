set(TEST_NAME "Test")

set(sources
  main.cpp
  test_io.cpp
  )

include_directories(${CMAKE_SOURCE_DIR}/src)

add_executable(${TEST_NAME} ${sources} $<TARGET_OBJECTS:lib${CMAKE_PROJECT_NAME}>)
add_dependencies(${TEST_NAME} lib${CMAKE_PROJECT_NAME})
#target_link_libraries(${TEST_NAME} ${IPOPT_LIBRARIES})

add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})

if(CMAKE_BUILD_TYPE STREQUAL "Coverage")
  include(EnableCoverage)
endif()
