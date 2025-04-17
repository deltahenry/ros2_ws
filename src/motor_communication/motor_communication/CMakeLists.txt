cmake_minimum_required(VERSION 3.8)
project(motor_communication)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(motor_info_interface REQUIRED)
# -----------------------------------------------------------------------------------------
# 若需要依賴非ROS的library，參考以下find_package來尋找
# find_package(PkgConfig  REQUIRED)
# pkg_check_modules(LIBMODBUS REQUIRED libmodbus)
# -----------------------------------------------------------------------------------------

# # 指定額外的include路徑
# include_directories(./include/${PROJECT_NAME})

# 生成可執行文件與其所需來源檔
# message("motor_communication_INCLUDE_DIRS: ${motor_communication_INCLUDE_DIRS}")
# add_executable(01_publisher
# src/01_publisher.cpp 
# ./include/${PROJECT_NAME}/classAPI_publisher.cpp	)
# ament_target_dependencies(01_publisher rclcpp std_msgs zj_00_beginner_interfaces)
# -----------------------------------------------------------------------------------------
# 若需要include&link非ROS的library，參考以下cmake的指令
# target_include_directories(01_publisher PUBLIC ${LIBMODBUS_INCLUDE_DIRS})
# target_link_libraries(01_publisher ${LIBMODBUS_LIBRARIES})
# -----------------------------------------------------------------------------------------

# add_executable(02_subscriber 
# 	src/02_subscriber.cpp
# 	./include/${PROJECT_NAME}/classAPI_subscriber.cpp	)
# ament_target_dependencies(02_subscriber rclcpp std_msgs zj_00_beginner_interfaces)


# add_executable(03_service_TwoIntAdd 
# src/03_service_two_int_add.cpp	)
# ament_target_dependencies(03_service_TwoIntAdd rclcpp example_interfaces)

# add_executable(04_client_TwoIntAdd 
# src/04_client_two_int_add.cpp	)
# ament_target_dependencies(04_client_TwoIntAdd rclcpp example_interfaces)

install(TARGETS
# 01_publisher
# 02_subscriber
# 03_service_TwoIntAdd
# 04_client_TwoIntAdd
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
