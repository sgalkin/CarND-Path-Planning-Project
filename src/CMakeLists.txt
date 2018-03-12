set(main main.cpp)
set(sources
  application.h
  compose.h
  config.h
  map.h
  point.h
  coordinates.h
  coordinates.cpp
  io.cpp
  io.h
  #   model.cpp
  lane.h
  model.h
  path.h
  planner.h
  planner.cpp

#   model_index.h
#   model_index.cpp
#   model_output.h
#   mpc.cpp
#   mpc.h
#   polynomial.h
#   processor.cpp
#   processor.h
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
