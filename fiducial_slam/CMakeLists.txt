cmake_minimum_required(VERSION 3.8)
project(fiducial_slam)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)

find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

find_package(visualization_msgs REQUIRED)

find_package(cv_bridge REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(fiducial_msgs REQUIRED)

find_package(image_transport REQUIRED)

find_package(std_srvs REQUIRED)

find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(OpenCV REQUIRED)

set(DEPENDENCIES
  rclcpp
  sensor_msgs
  std_msgs
  visualization_msgs
  fiducial_msgs
  geometry_msgs
  cv_bridge
  tf2_ros
  image_transport
  tf2_geometry_msgs
  std_srvs
)

# -------------- Include headers --------------
include_directories(
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

add_executable(fiducial_slam_node 
                src/fiducial_slam.cpp 
                src/map.cpp
                src/transform_with_variance.cpp
                src/observation.cpp)
ament_target_dependencies(fiducial_slam_node ${DEPENDENCIES} OpenCV)

# -------------- Install --------------
install(
        DIRECTORY include/
        DESTINATION include/
        FILES_MATCHING PATTERN "*.h*"
        PATTERN ".git" EXCLUDE)

install(DIRECTORY
        launch
        resources
        DESTINATION share/${PROJECT_NAME}
        )

install(TARGETS
        fiducial_slam_node
        DESTINATION lib/${PROJECT_NAME})

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

ament_package()
