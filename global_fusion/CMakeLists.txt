cmake_minimum_required(VERSION 3.10)
project(global_fusion)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Ceres REQUIRED)

add_subdirectory(./ThirdParty/GeographicLib/)

include_directories(
  ${CERES_INCLUDE_DIRS}
  ./ThirdParty/GeographicLib/include/
)

add_executable(global_fusion_node
	src/globalOptNode.cpp
	src/globalOpt.cpp)

ament_target_dependencies(global_fusion_node rclcpp nav_msgs geometry_msgs sensor_msgs visualization_msgs rclpy)
target_link_libraries(global_fusion_node ${catkin_LIBRARIES} ${CERES_LIBRARIES} libGeographiccc)

# Install
install(TARGETS
    global_fusion_node
    DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS global_fusion_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
install(TARGETS global_fusion_node
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()