cmake_minimum_required(VERSION 3.16)
project(nav2_imm_ppo_critic)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_mppi_controller 1.1.12 EXACT REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/InteractionContext.msg"
  DEPENDENCIES std_msgs)
rosidl_get_typesupport_target(context_typesupport ${PROJECT_NAME} rosidl_typesupport_cpp)

add_library(nav2_imm_ppo_interaction_critic SHARED src/interaction_critic.cpp)
target_include_directories(nav2_imm_ppo_interaction_critic PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
# The installed Nav2 package exports mppi_controller/mppi_critics, not a
# library named nav2_mppi_controller. Use its ament dependency exports.
ament_target_dependencies(nav2_imm_ppo_interaction_critic
  rclcpp rclcpp_lifecycle std_msgs pluginlib nav2_costmap_2d nav2_mppi_controller)
target_link_libraries(nav2_imm_ppo_interaction_critic "${context_typesupport}")
pluginlib_export_plugin_description_file(nav2_mppi_controller critic_plugins.xml)

install(TARGETS nav2_imm_ppo_interaction_critic
  EXPORT export_nav2_imm_ppo_interaction_critic
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_interaction_critic test/test_interaction_critic.cpp TIMEOUT 120)
  if(TARGET test_interaction_critic)
    target_link_libraries(test_interaction_critic
      nav2_imm_ppo_interaction_critic "${context_typesupport}")
    ament_target_dependencies(test_interaction_critic
      rclcpp rclcpp_lifecycle pluginlib nav2_mppi_controller)
  endif()
endif()

ament_export_targets(export_nav2_imm_ppo_interaction_critic HAS_LIBRARY_TARGET)
ament_export_dependencies(rosidl_default_runtime rosidl_typesupport_cpp
  rclcpp rclcpp_lifecycle std_msgs pluginlib nav2_costmap_2d nav2_mppi_controller)
ament_package()
