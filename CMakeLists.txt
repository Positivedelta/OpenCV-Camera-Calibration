cmake_minimum_required(VERSION 3.16)

project(camera-calibration)

add_definitions(-DAPI_EXPORTS)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

include_directories(${PROJECT_SOURCE_DIR}/include)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

# notes 1, setup the install base path for the camera calibration application
#       2, choose the /usr/local/bit-parallel path once ready to deploy in production (or edit to suit your needs)
#
#set(BIT_PARALLEL_INSTALL_ROOT "/usr/local/bit-parallel")
set(BIT_PARALLEL_INSTALL_ROOT "${PROJECT_SOURCE_DIR}/bit-parallel")

# the make target, build the application
#
add_executable(camera-calibration-bp-v1.0 ${PROJECT_SOURCE_DIR}/src/camera-calibration.cpp ${SRCS})
target_link_libraries(camera-calibration-bp-v1.0 ${OpenCV_LIBS})

# the install target
#
install(TARGETS camera-calibration-bp-v1.0 DESTINATION ${BIT_PARALLEL_INSTALL_ROOT}/bin/camera)
