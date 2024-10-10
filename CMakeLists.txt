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
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(acomms_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(dependencies
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  stonefish_ros2
  nav_msgs
  acomms_msgs
)

include_directories(include
  ${EIGEN3_INCLUDE_DIR}
)
set(library_name ${PROJECT_NAME})

add_library(${library_name} SHARED 
  include/world_of_stonefish/imu_driver.cpp
  include/world_of_stonefish/thruster_driver.cpp
  include/world_of_stonefish/dvl_driver.cpp
  include/world_of_stonefish/pressure_sensor_driver.cpp
  include/world_of_stonefish/servo_driver.cpp
  include/world_of_stonefish/usbl_driver.cpp
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

add_executable(thruster_driver_node src/thruster_driver_node.cpp)
target_link_libraries(thruster_driver_node ${library_name})
ament_target_dependencies(thruster_driver_node ${dependencies})

install(TARGETS thruster_driver_node ${library_name}
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
RUNTIME DESTINATION lib/${PROJECT_NAME}
)

add_executable(dvl_driver_node src/dvl_driver_node.cpp)
target_link_libraries(dvl_driver_node ${library_name})
ament_target_dependencies(dvl_driver_node ${dependencies})

install(TARGETS dvl_driver_node ${library_name}
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
RUNTIME DESTINATION lib/${PROJECT_NAME}
)

add_executable(pressure_sensor_node src/pressure_sensor_node.cpp)
target_link_libraries(pressure_sensor_node ${library_name})
ament_target_dependencies(pressure_sensor_node ${dependencies})

install(TARGETS pressure_sensor_node ${library_name}
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
RUNTIME DESTINATION lib/${PROJECT_NAME}
)


add_executable(servo_driver_node src/servo_driver_node.cpp)
target_link_libraries(servo_driver_node ${library_name})
ament_target_dependencies(servo_driver_node ${dependencies})

install(TARGETS servo_driver_node ${library_name}
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
RUNTIME DESTINATION lib/${PROJECT_NAME}
)

add_executable(usbl_driver_node src/usbl_driver_node.cpp)
target_link_libraries(usbl_driver_node ${library_name})
ament_target_dependencies(usbl_driver_node ${dependencies})

install(TARGETS usbl_driver_node ${library_name}
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