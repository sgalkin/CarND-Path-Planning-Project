set(main main.cpp)
set(sources
  application.h
  compose.h
  config.h
  control.h
  control.cpp
  coordinates.cpp
  coordinates.h
  dump.h
  io.cpp
  io.h
  fusion.cpp
  fusion.h
  lane.cpp
  lane.h
  limits.h
  map.h
  model.h
  path.h
  planner.cpp
  planner.h
  point.h
  protocol.cpp
  protocol.h
  util.h
)

add_library(lib${CMAKE_PROJECT_NAME} OBJECT ${sources})
add_executable(${CMAKE_PROJECT_NAME} ${main} $<TARGET_OBJECTS:lib${CMAKE_PROJECT_NAME}>)
target_link_libraries(${CMAKE_PROJECT_NAME}
  ${ZLIB_LIBRARIES}
  ${UWS_LIBRARIES}
  )

set_target_properties(
  ${CMAKE_PROJECT_NAME}
  PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
  )
