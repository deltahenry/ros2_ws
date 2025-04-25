cmake_minimum_required(VERSION 3.8)
project(motor_communication)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(motor_info_interface REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(uros_interface REQUIRED)
# -----------------------------------------------------------------------------------------
# 若需要依賴非ROS的library，參考以下find_package來尋找
# find_package(PkgConfig  REQUIRED)
# pkg_check_modules(LIBMODBUS REQUIRED libmodbus)
# -----------------------------------------------------------------------------------------

# 指定額外的include路徑
include_directories(./include/${PROJECT_NAME})

# 生成可執行文件與其所需來源檔
# message("motor_communication_INCLUDE_DIRS: ${motor_communication_INCLUDE_DIRS}")
# -----------------------------------------------------------------------------------------
# # 若需要include&link非ROS的library，參考以下cmake的指令
# target_include_directories(node_name PUBLIC ${LIBMODBUS_INCLUDE_DIRS})
# target_link_libraries(node_name ${LIBMODBUS_LIBRARIES})
# -----------------------------------------------------------------------------------------

add_executable(motor_servo_ctl 
	src/motor_servo_ctl_node.cpp
	./include/${PROJECT_NAME}/classAPI_motor_servo_ctl.cpp	)
ament_target_dependencies(motor_servo_ctl rclcpp std_msgs custom_msgs uros_interface)


add_executable(03_service_TwoIntAdd 
src/03_service_two_int_add.cpp	)
ament_target_dependencies(03_service_TwoIntAdd rclcpp uros_interface)

install(TARGETS
motor_servo_ctl
03_service_TwoIntAdd
DESTINATION lib/${PROJECT_NAME} )

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
