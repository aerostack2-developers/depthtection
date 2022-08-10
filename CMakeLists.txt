cmake_minimum_required(VERSION 3.8)
project(depthtection)
project(${PROJECT_NAME} VERSION 0.0.0)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# set Release as default
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

#set fPIC to ON by default
#set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# find dependencies
set(PROJECT_DEPENDENCIES
ament_cmake
ament_cmake_ros
as2_core
as2_msgs
rclcpp 
sensor_msgs 
OpenCV 
cv_bridge 
tf2 
tf2_ros 
tf2_msgs 
tf2_geometry_msgs
nav_msgs 
vision_msgs
)
foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

include_directories(
  include
  include/${PROJECT_NAME}
)

set(SOURCE_FILES
  src/depthtection.cpp
)

add_executable(${PROJECT_NAME}_node src/depthtection_node.cpp ${SOURCE_FILES})
# target_link_libraries(${PROJECT_NAME}_node yaml-cpp)
target_include_directories(${PROJECT_NAME}_node 
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}_node ${PROJECT_DEPENDENCIES})

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

# install(DIRECTORY
#   config
#   DESTINATION share/${PROJECT_NAME}
# )

ament_package()
