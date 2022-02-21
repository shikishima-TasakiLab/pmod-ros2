cmake_minimum_required(VERSION 3.5)
project(pmod_ros)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(message_filters REQUIRED)

set(Caffe2_DIR "/opt/libtorch/share/cmake/Caffe2")
set(Torch_DIR "/opt/libtorch/share/cmake/Torch")
find_package(Torch REQUIRED)
find_package(PCL REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(pmod src/yaml.cpp src/pmod.cpp src/pmod_node.cpp)
ament_target_dependencies(pmod
  rclcpp
  tf2_ros
  cv_bridge
  pcl_conversions
  message_filters
)
target_link_libraries(pmod ${TORCH_LIBRARIES} ${PCL_LIBRARIES})

link_directories(
  ${PCL_LIBRARY_DIRS}
)
add_definitions(
  ${PCL_DEFINITIONS}
)

install(TARGETS
  pmod
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
