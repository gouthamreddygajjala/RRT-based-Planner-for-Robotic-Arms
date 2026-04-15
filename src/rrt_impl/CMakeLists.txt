cmake_minimum_required(VERSION 3.8)
project(rrt_impl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ================= Dependencies =================

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(hpp-fcl REQUIRED)
find_package(ruckig REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


# ================= Include =================

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# ================= Library =================

add_library(rrt_core
  src/rrt.cpp
  src/rrt_star.cpp
  src/rrt_connect.cpp
  src/collision.cpp
)

# Link libraries
target_link_libraries(rrt_core
  pinocchio::pinocchio
  hpp-fcl::hpp-fcl
  ruckig::ruckig       
)

# ROS dependencies
ament_target_dependencies(rrt_core
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  visualization_msgs
  custom_msgs
  trajectory_msgs
  sensor_msgs
)

add_executable(rrt_node src/rrt_node.cpp)

target_link_libraries(rrt_node rrt_core)

ament_target_dependencies(rrt_node rclcpp)


# ================= Install =================

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  TARGETS
    rrt_core
    rrt_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# ================= Testing =================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()