cmake_minimum_required(VERSION 3.10)
project(mpt_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# find_package(message_package REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# find_package(message_generation REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(OpenMP REQUIRED)
# find_package(FCL REQUIRED)
find_package(Assimp REQUIRED)

find_package(fcl 0.6 QUIET)
if(fcl_FOUND)
  set(FCL_LIBRARIES fcl)
  set(using_new_fcl true)
  message(STATUS "Using FCL version: ${FCL_VERSION}")
else()
  include(FindPkgConfig)
  pkg_check_modules(PC_FCL REQUIRED fcl)
  set(FCL_LIBRARIES ${PC_FCL_LIBRARIES} ${PC_CCD_LIBRARIES})
  set(using_new_fcl)
  message(STATUS "Using FCL version: ${PC_FCL_VERSION}")
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

set(msg_files
  "msg/MotionPlanRequest.msg"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)
ament_export_dependencies(rosidl_default_runtime)

include_directories(
  include
  external/nigh/src
  external/mpt/src
)

add_executable(${PROJECT_NAME}_node src/mpt_ros_node.cpp)
add_executable(${PROJECT_NAME}_client src/mpt_ros_client.cpp)
add_executable(${PROJECT_NAME}_fetch_node src/mpt_ros_fetch_node.cpp)
add_executable(${PROJECT_NAME}_fetch_client src/mpt_ros_fetch_client.cpp)

set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
set_target_properties(${PROJECT_NAME}_client PROPERTIES OUTPUT_NAME client PREFIX "")
set_target_properties(${PROJECT_NAME}_fetch_node PROPERTIES OUTPUT_NAME fetch_node PREFIX "")
set_target_properties(${PROJECT_NAME}_fetch_client PROPERTIES OUTPUT_NAME fetch_client PREFIX "")

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(${PROJECT_NAME}_node
  Eigen3::Eigen
  ${FCL_LIBRARIES}
  ${ASSIMP_LIBRARIES}
  ${cpp_typesupport_target}
  )
target_link_libraries(${PROJECT_NAME}_client
  Eigen3::Eigen
  ${cpp_typesupport_target}  
  )
target_link_libraries(${PROJECT_NAME}_fetch_node
  Eigen3::Eigen
  ${FCL_LIBRARIES}
  ${ASSIMP_LIBRARIES}
  ${cpp_typesupport_target}
  )
target_link_libraries(${PROJECT_NAME}_fetch_client
  Eigen3::Eigen
  ${cpp_typesupport_target}
  )

ament_target_dependencies(${PROJECT_NAME}_client
  rclcpp
  rclpy
  std_msgs
  geometry_msgs)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  rclpy
  std_msgs
  geometry_msgs)

ament_target_dependencies(${PROJECT_NAME}_fetch_client
  rclcpp
  rclpy
  std_msgs
  geometry_msgs)

ament_target_dependencies(${PROJECT_NAME}_fetch_node
  rclcpp
  rclpy
  std_msgs
  geometry_msgs)

ament_package()
