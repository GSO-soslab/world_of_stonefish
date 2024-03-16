cmake_minimum_required(VERSION 3.8)
project(world_of_stonefish)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(stonefish_ros2 REQUIRED)
find_package(Stonefish REQUIRED 1.3.0)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(dependencies
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
)

include_directories(include
  ${EIGEN3_INCLUDE_DIR}
)
set(library_name ${PROJECT_NAME})

add_library(${library_name} SHARED 
src/imu_driver.cpp
src/actuator_driver.cpp
)


ament_target_dependencies(
  ${library_name} 
  ${dependencies} 
  tf2
  tf2_ros
)

add_executable(imu_driver_node src/imu_driver_node.cpp)
target_link_libraries(imu_driver_node ${library_name})
ament_target_dependencies(imu_driver_node ${dependencies})

install(TARGETS imu_driver_node ${library_name}
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
RUNTIME DESTINATION lib/${PROJECT_NAME}
)

add_executable(actuator_driver_node src/actuator_driver_node.cpp)
target_link_libraries(actuator_driver_node ${library_name})
ament_target_dependencies(actuator_driver_node ${dependencies})

install(TARGETS actuator_driver_node ${library_name}
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
RUNTIME DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY include/
DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

##launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY 
  data 
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY 
  metadata 
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY 
  vehicles 
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY 
  world 
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY 
  config 
  DESTINATION share/${PROJECT_NAME}
)

ament_package()