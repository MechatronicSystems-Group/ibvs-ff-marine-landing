cmake_minimum_required(VERSION 3.8)
project(aruco)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)

find_package(px4_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(custom_msgs REQUIRED)

# NOTE: Build opencv from source, hence the path to /usr/local/lib
find_package(OpenCV 4.10.0 REQUIRED PATHS /usr/local NO_DEFAULT_PATH)

include_directories(${Eigen3_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${cv_bridge_INCLUDE_DIRS})
link_directories(/usr/local/lib)

add_executable(aruco_node src/aruco_node.cpp)
ament_target_dependencies(aruco_node rclcpp sensor_msgs geometry_msgs cv_bridge px4_msgs custom_msgs)


target_link_libraries(aruco_node ${OpenCV_LIBS})

target_include_directories(aruco_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(aruco_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS aruco_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

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

install(DIRECTORY cfg
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
