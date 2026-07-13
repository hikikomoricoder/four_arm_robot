cmake_minimum_required(VERSION 3.8)
project(robot_commander)

# Ignore Conda paths to prevent OpenSSL/libcrypto version mismatches
# during linking (e.g., Conda's OpenSSL 3.2.x vs Ubuntu's 3.0.x).
if(DEFINED ENV{CONDA_PREFIX})
  list(APPEND CMAKE_IGNORE_PREFIX_PATH "$ENV{CONDA_PREFIX}")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Force system fmt to prevent Conda's libfmt from leaking into the link line.
# (The Conda fmt target pulls in Conda include/library paths.)
set(fmt_DIR "/usr/lib/x86_64-linux-gnu/cmake/fmt")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(my_robot_interfaces REQUIRED)

add_executable(test_moveit src/test_moveit.cpp)
ament_target_dependencies(test_moveit rclcpp moveit_ros_planning_interface)

# Ensure the linker resolves transitive dependencies (e.g. libssl.so.3)
# against the system libraries before falling back to LD_LIBRARY_PATH,
# which contains the Conda path.
target_link_options(test_moveit PRIVATE "-Wl,-rpath-link,/usr/lib/x86_64-linux-gnu")

# ── Library (reusable by other packages / nodes) ──────────────────
add_library(wheel_commander_lib SHARED
  src/wheel_commander.cpp
)
target_include_directories(wheel_commander_lib PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
ament_target_dependencies(wheel_commander_lib
  rclcpp rclcpp_action control_msgs sensor_msgs)
target_link_options(wheel_commander_lib PRIVATE "-Wl,-rpath-link,/usr/lib/x86_64-linux-gnu")

# ── Executable ────────────────────────────────────────────────────
add_executable(wheel_command
  src/wheel_command.cpp
)
ament_target_dependencies(wheel_command
  rclcpp rclcpp_action control_msgs sensor_msgs)
# wheel_commander_lib must be linked with plain signature to stay
# compatible with the plain signature used internally by
# ament_target_dependencies above.
target_link_libraries(wheel_command wheel_commander_lib)
# Keep wheel_commander_lib discoverable at runtime (both installed in
# lib/${PROJECT_NAME}/).  $ORIGIN resolves to the directory of the
# executable itself.
set_target_properties(wheel_command PROPERTIES
  INSTALL_RPATH "\$ORIGIN"
)
target_link_options(wheel_command PRIVATE "-Wl,-rpath-link,/usr/lib/x86_64-linux-gnu")

# ── Install ────────────────────────────────────────────────────────
install(TARGETS
  wheel_commander_lib
  test_moveit
  wheel_command
  EXPORT ${PROJECT_NAME}Targets
  DESTINATION lib/${PROJECT_NAME}/
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_targets(${PROJECT_NAME}Targets)
ament_export_dependencies(
  rclcpp rclcpp_action control_msgs sensor_msgs)

ament_package()
