cmake_minimum_required(VERSION 2.6)
project(vslam)

add_compile_options(-std=c++11)
SET(CMAKE_BUILD_TYPE Release)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

find_package(OpenCV 4 REQUIRED)
find_package(Sophus REQUIRED)

include_directories(
    ${PROJECT_SOURCE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${Sophus_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/bundle_adjustment/include
    ${PROJECT_SOURCE_DIR}/core/include
)

add_library(${PROJECT_NAME}_lib SHARED
    bundle_adjustment/src/mappoint.cpp
    bundle_adjustment/src/camera.cpp
    bundle_adjustment/src/cost_function.cpp
    bundle_adjustment/src/bundle_adjustment.cpp
)

target_link_libraries(${PROJECT_NAME}_lib
    ${EIGEN3_LIBS}
    ${OpenCV_LIBS}
    ${Sophus_LIBRARIES}
)


add_executable(bundle_adjustment bundle_adjustment/test/test_bundle_adjustment.cpp)
target_link_libraries(bundle_adjustment
	${PROJECT_NAME}_lib
)



