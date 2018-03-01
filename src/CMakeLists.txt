set(main main.cpp)
set(sources
  application.h
  compose.h
  config.h
  map.h
  point.h
  frenet.h
  frenet.cpp
#   control.h
#   drive.h
  io.cpp
  io.h
#   model.cpp
#   model.h
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
#   solve.h
#   solve_traits.h
#   state.h
#   stream_holder.h
#   util.h
   )

add_library(lib${CMAKE_PROJECT_NAME} OBJECT ${sources})
add_executable(${CMAKE_PROJECT_NAME} ${main} $<TARGET_OBJECTS:lib${CMAKE_PROJECT_NAME}>)
target_link_libraries(${CMAKE_PROJECT_NAME}
  ${ZLIB_LIBRARIES}
#  ${CPPAD_LIBRARIES}
  ${UWS_LIBRARIES}
#  ${IPOPT_LIBRARIES}
  )

set_target_properties(
  ${CMAKE_PROJECT_NAME}
  PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
  )
