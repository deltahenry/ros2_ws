cmake_minimum_required(VERSION 3.8)
project(custom_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Message files
set(msg_files
  "msg/PoseIncrement.msg"
  "msg/ButtonCmd.msg"
  "msg/StateInfo.msg"  
  "msg/Finished.msg"  
  "msg/InterfaceSingleMotor.msg"# <- Corrected typo
  "msg/InterfaceMultipleMotors.msg"# <- Corrected typo
)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(rosidl_default_generators)

# Linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
