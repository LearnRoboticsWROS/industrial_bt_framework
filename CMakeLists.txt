cmake_minimum_required(VERSION 3.8)
project(industrial_bt_framework)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Dependencies ─────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ── Service interfaces ───────────────────────────────────────────────────────
rosidl_generate_interfaces(${PROJECT_NAME}
  srv/RemoveSceneObject.srv
  srv/SetScenePose.srv
  DEPENDENCIES geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)

# ── BehaviorTree.CPP v4 ───────────────────────────────────────────────────────
find_package(behaviortree_cpp QUIET)
if(NOT behaviortree_cpp_FOUND)
  message(STATUS
    "[industrial_bt_framework] behaviortree_cpp not found via find_package. "
    "Fetching BehaviorTree.CPP from GitHub (requires internet at configure time).")
  include(FetchContent)
  FetchContent_Declare(
    behaviortree_cpp
    GIT_REPOSITORY https://github.com/BehaviorTree/BehaviorTree.CPP.git
    GIT_TAG        master
    GIT_SHALLOW    TRUE
  )
  set(BTCPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
  set(BTCPP_EXAMPLES    OFF CACHE BOOL "" FORCE)
  set(BTCPP_UNIT_TESTS  OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(behaviortree_cpp)
  set(BTCPP_FETCHED TRUE)
else()
  message(STATUS "[industrial_bt_framework] Using installed behaviortree_cpp.")
  set(BTCPP_FETCHED FALSE)
endif()

# ── yaml-cpp (via ROS 2 vendor package) ──────────────────────────────────────
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

# ── Common deps list (reused) ────────────────────────────────────────────────
set(COMMON_DEPS
  rclcpp
  moveit_ros_planning_interface
  geometric_shapes
  shape_msgs
  moveit_msgs
  geometry_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pluginlib
  yaml_cpp_vendor
)

# Helper macro that links BT.CPP to a target.
function(link_btcpp tgt)
  if(BTCPP_FETCHED)
    target_link_libraries(${tgt} behaviortree_cpp)
  else()
    if(TARGET behaviortree_cpp::behaviortree_cpp)
      target_link_libraries(${tgt} behaviortree_cpp::behaviortree_cpp)
    else()
      target_link_libraries(${tgt} behaviortree_cpp)
    endif()
  endif()
endfunction()

# ── MoveIt Cartesian backend plugin (shared library) ─────────────────────────
add_library(industrial_bt_backends SHARED
  src/backends/moveit_cartesian_backend.cpp
)
target_include_directories(industrial_bt_backends PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(industrial_bt_backends ${COMMON_DEPS})
target_link_libraries(industrial_bt_backends yaml-cpp)

# ── bt_runner_node executable ─────────────────────────────────────────────────
add_executable(bt_runner_node
  src/robot_client.cpp
  src/tool_registry.cpp
  src/scene_client.cpp
  src/perception_client.cpp
  src/bt_nodes.cpp
  src/bt_runner_node.cpp
)
target_include_directories(bt_runner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(bt_runner_node ${COMMON_DEPS})
target_link_libraries(bt_runner_node yaml-cpp)
link_btcpp(bt_runner_node)
# Link in-package srv interfaces
rosidl_get_typesupport_target(IBT_SRV_TYPESUPPORT ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(bt_runner_node ${IBT_SRV_TYPESUPPORT})

# ── ZMQ detection for bt_runner_node_visual ───────────────────────────────────
find_package(PkgConfig QUIET)
set(BTCPP_GROOT2_SUPPORT OFF)
if(PkgConfig_FOUND)
  pkg_check_modules(ZMQ QUIET libzmq)
  if(ZMQ_FOUND)
    set(BTCPP_GROOT2_SUPPORT ON)
    message(STATUS "[industrial_bt_framework] libzmq found — Groot2Publisher ENABLED.")
  else()
    message(STATUS "[industrial_bt_framework] libzmq not found — Groot2Publisher DISABLED.")
  endif()
else()
  find_library(ZMQ_LIB zmq)
  if(ZMQ_LIB)
    set(BTCPP_GROOT2_SUPPORT ON)
    set(ZMQ_LIBRARIES ${ZMQ_LIB})
    message(STATUS "[industrial_bt_framework] libzmq found via find_library — Groot2Publisher ENABLED.")
  endif()
endif()

# ── bt_runner_node_visual executable ─────────────────────────────────────────
add_executable(bt_runner_node_visual
  src/robot_client.cpp
  src/tool_registry.cpp
  src/scene_client.cpp
  src/perception_client.cpp
  src/bt_nodes.cpp
  src/bt_runner_node_visual.cpp
)
target_include_directories(bt_runner_node_visual PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(bt_runner_node_visual ${COMMON_DEPS})
target_link_libraries(bt_runner_node_visual yaml-cpp)
link_btcpp(bt_runner_node_visual)
target_link_libraries(bt_runner_node_visual ${IBT_SRV_TYPESUPPORT})

if(BTCPP_GROOT2_SUPPORT)
  target_compile_definitions(bt_runner_node_visual PRIVATE BTCPP_GROOT2_SUPPORT)
  if(ZMQ_LIBRARIES)
    target_link_libraries(bt_runner_node_visual ${ZMQ_LIBRARIES})
  endif()
endif()

# ── scene_manager_node executable ────────────────────────────────────────────
add_executable(scene_manager_node
  src/scene_manager_node.cpp
)
target_include_directories(scene_manager_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(scene_manager_node ${COMMON_DEPS})
target_link_libraries(scene_manager_node yaml-cpp)
target_link_libraries(scene_manager_node ${IBT_SRV_TYPESUPPORT})

# ── Pluginlib export ─────────────────────────────────────────────────────────
pluginlib_export_plugin_description_file(industrial_bt_framework plugins/industrial_bt_plugins.xml)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS
    industrial_bt_backends
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
    bt_runner_node
    bt_runner_node_visual
    scene_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config bt_trees doc plugins
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(industrial_bt_backends)
ament_export_dependencies(${COMMON_DEPS} behaviortree_cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
