configure_file(${CMAKE_SOURCE_DIR}/Dockerfile.in ${CMAKE_BINARY_DIR}/Dockerfile)
file(WRITE ${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}
  "#/usr/bin/env bash\n\n"
  "docker run --rm -ti ${DOCKER_NETWORK} ${DOCKER_RELEASE_IMAGE} $@\n")

set(DOCKER_RELEASE_IMAGE sgalkin/carnd-${CMAKE_PROJECT_NAME})
set(DOCKER_RELEASE_BUILD
  docker build
  --pull
  --force-rm
  -t ${DOCKER_RELEASE_IMAGE}
  -f ${CMAKE_BINARY_DIR}/Dockerfile
  ${CMAKE_SOURCE_DIR})

set(DOCKER_RELEASE_TAG `git rev-parse --abbrev-ref HEAD`.`git rev-parse --short HEAD`)

set(DOCKER_DEV_IMAGE sgalkin/carnd-t2-dev)
set(DOCKER_NAME cmake-${CMAKE_PROJECT_NAME}-env)
set(DOCKER_MOUNT -v ${CMAKE_SOURCE_DIR}:/repo:ro)
set(DOCKER_NETWORK -p4567:4567)
set(DOCKER_RUN docker run --rm -d -ti ${DOCKER_MOUNT} ${DOCKER_NETWORK} --name ${DOCKER_NAME} ${DOCKER_DEV_IMAGE})
set(DOCKER_TERM -e TERM='$ENV{TERM}')
set(DOCKER_EXEC docker exec -ti ${DOCKER_TERM})

add_custom_target(start-docker-env
  COMMAND docker ps -f name=${DOCKER_NAME} | grep -q ${DOCKER_NAME} || (docker pull ${DOCKER_DEV_IMAGE} && ${DOCKER_RUN}))

add_custom_target(stop-docker-env
  COMMAND docker stop ${DOCKER_NAME})

add_custom_target(docker-clean
  COMMAND ${DOCKER_EXEC} ${DOCKER_NAME} cmake --build . --target clean
  DEPENDS start-docker-env)

add_custom_target(docker-build
  COMMAND ${DOCKER_EXEC} ${DOCKER_NAME} sh -c \"test -f CMakeCache.txt || cmake /repo -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} -GNinja\"
  COMMAND ${DOCKER_EXEC} ${DOCKER_NAME} cmake --build . 
  DEPENDS start-docker-env)

add_custom_target(docker-test
  COMMAND ${DOCKER_EXEC} ${DOCKER_NAME} ctest --output-on-failure
  DEPENDS docker-build)

add_custom_target(docker-run
  COMMAND ${DOCKER_EXEC} ${DOCKER_NAME} ./${CMAKE_PROJECT_NAME}
  DEPENDS docker-build)

add_custom_target(docker-shell
  COMMAND ${DOCKER_EXEC} -ti ${DOCKER_TERM} ${DOCKER_NAME} bash
  DEPENDS start-docker-env)

if(NOT ${LOCAL_BUILD})
  add_custom_target(default ALL DEPENDS docker-build docker-test)
endif()

add_custom_target(docker-release
  COMMAND git diff-index --quiet HEAD || (echo "There are uncommited changes. Please commit to continue" && false)
  COMMAND ${DOCKER_RELEASE_BUILD}
  COMMAND docker tag ${DOCKER_RELEASE_IMAGE} ${DOCKER_RELEASE_IMAGE}:${DOCKER_RELEASE_TAG}
  COMMAND chmod 755 ${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}
  DEPENDS ${CMAKE_BINARY_DIR}/Dockerfile ${CMAKE_SOURCE_DIR}/Dockerfile.in
  SOURCES ${CMAKE_SOURCE_DIR}/Dockerfile.in)

add_custom_target(docker-push
  COMMAND docker push ${DOCKER_RELEASE_IMAGE}:${DOCKER_RELEASE_TAG})
