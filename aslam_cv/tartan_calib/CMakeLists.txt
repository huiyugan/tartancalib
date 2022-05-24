cmake_minimum_required(VERSION 3.0.2)
project(tartan_calib)

set(CMAKE_CXX_STANDARD 14)

find_package(Boost REQUIRED COMPONENTS serialization) 
find_package(catkin REQUIRED COMPONENTS
  sm_common
  sm_eigen
  sm_boost
  sm_kinematics
  aslam_time
  sm_opencv
  opencv2_catkin
  aslam_cameras
  sm_property_tree
  aslam_time
)


catkin_package(
  INCLUDE_DIRS include
)

add_definitions(-fPIC -Wextra -Winit-self -Woverloaded-virtual -Wnon-virtual-dtor -Wsign-promo -Wno-long-long -Wl,--no-allow-shlib-undefined)

set(OTHER_LIBS)
if(NOT APPLE)
  set(OTHER_LIBS rt)
endif()

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/TartanCalibWorker.cpp
)

target_compile_options(${PROJECT_NAME} PUBLIC "-Wno-deprecated-copy") # TODO: Fix...
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${OTHER_LIBS})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)