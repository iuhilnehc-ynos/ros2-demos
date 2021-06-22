include_directories(include)

add_library(custom_talker_component SHARED
  src/custom_talker_component.cpp)
target_compile_definitions(custom_talker_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(custom_talker_component
  "rclcpp"
  "rclcpp_components"
  "std_msgs")
rclcpp_components_register_nodes(custom_talker_component "composition::CustomTalker")

# Just a workaround refered to
# https://answers.ros.org/question/361289/ros2-components-registration-from-subdirectory-cmakelists-file/?answer=361439#post-id-361439
set(_RCLCPP_COMPONENTS_${resource_index}__NODES
  "${_RCLCPP_COMPONENTS_${resource_index}__NODES}" PARENT_SCOPE)

install(TARGETS
  custom_talker_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


