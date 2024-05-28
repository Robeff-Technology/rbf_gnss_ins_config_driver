cmake_minimum_required(VERSION 3.8)
project(rbf_gnss_ins_config_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

set(msg_files
    "srv/Reset.srv"
    "srv/ChangeBaud.srv"
    "srv/GetConfig.srv"
    "srv/SetConfig.srv"
    "srv/WriteCommand.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
    DEPENDENCIES
    std_srvs
    std_msgs
)

add_library(gnss_ins_config_component SHARED
  src/rbf_gnss_ins_config.cpp
  src/serial_port.cpp
)

ament_target_dependencies(gnss_ins_config_component
  rclcpp
  rclcpp_components
  std_srvs
  std_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(gnss_ins_config_component "${cpp_typesupport_target}") 

rclcpp_components_register_node(
  gnss_ins_config_component
  PLUGIN "rbf_gnss_ins_config_driver::ConfigGnssIns"
  EXECUTABLE rbf_gnss_ins_config
)

ament_export_targets(export_gnss_ins_config_component)
install(TARGETS gnss_ins_config_component
        EXPORT export_gnss_ins_config_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME}
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

ament_package()
