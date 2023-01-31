cmake_minimum_required(VERSION 3.0.2)
project(tiago_iaslab_simulation)

# # Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# # Find catkin macros and libraries
# # if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# # is used, also find other catkin packages
find_package(catkin REQUIRED
  gazebo_ros
  roscpp
  std_msgs
  message_generation
  actionlib
  tf2
  tf2_ros
  tf2_geometry_msgs
  move_base_msgs
  costmap_2d
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  apriltag_ros
  control_msgs
)

# # System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

# # Uncomment this if the package has a setup.py. This macro ensures
# # modules and global scripts declared therein get installed
# # See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

# ###############################################
# # Declare ROS messages, services and actions ##
# ###############################################

# # To declare and build messages, services or actions from within this
# # package, follow these steps:
# # * Let MSG_DEP_SET be the set of packages whose message types you use in
# #   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
# # * In the file package.xml:
# #   * add a build_depend tag for "message_generation"
# #   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
# #   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
# #     but can be declared for certainty nonetheless:
# #     * add a exec_depend tag for "message_runtime"
# # * In this file (CMakeLists.txt):
# #   * add "message_generation" and every package in MSG_DEP_SET to
# #     find_package(catkin REQUIRED COMPONENTS ...)
# #   * add "message_runtime" and every package in MSG_DEP_SET to
# #     catkin_package(CATKIN_DEPENDS ...)
# #   * uncomment the add_*_files sections below as needed
# #     and list every .msg/.srv/.action file to be processed
# #   * uncomment the generate_messages entry below
# #   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

# # Generate messages in the 'msg' folder
add_message_files(
  FILES
  circle.msg
)

# # Generate services in the 'srv' folder
add_service_files(
  FILES
  scanObstacles.srv
  Objs.srv
  apriltagDetections.srv
  Colors.srv
)

# # Generate actions in the 'action' folder
add_action_files(
  FILES
  moveScan.action
  head.action
  pick.action
  place.action
)

# # Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
  geometry_msgs
  apriltag_ros
)

# ###############################################
# # Declare ROS dynamic reconfigure parameters ##
# ###############################################

# # To declare and build dynamic reconfigure parameters within this
# # package, follow these steps:
# # * In the file package.xml:
# #   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
# # * In this file (CMakeLists.txt):
# #   * add "dynamic_reconfigure" to
# #     find_package(catkin REQUIRED COMPONENTS ...)
# #   * uncomment the "generate_dynamic_reconfigure_options" section below
# #     and list every .cfg file to be processed

# # Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
# cfg/DynReconf1.cfg
# cfg/DynReconf2.cfg
# )

# ##################################
# # catkin specific configuration ##
# ##################################
# # The catkin_package macro generates cmake config files for your package
# # Declare things to be passed to dependent projects
# # INCLUDE_DIRS: uncomment this if your package contains header files
# # LIBRARIES: libraries you create in this project that dependent projects also need
# # CATKIN_DEPENDS: catkin_packages dependent projects also need
# # DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(

  # INCLUDE_DIRS include
  # LIBRARIES tiago_iaslab_simulation
  # CATKIN_DEPENDS other_catkin_pkg
  # DEPENDS system_lib
  CATKIN_DEPENDS
  actionlib
  roscpp
  std_msgs
  message_runtime

  # DEPENDS system_lib
)

# ##########
# # Build ##
# ##########

# # Specify additional locations of header files
# # Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# # Declare a C++ library
add_library(${PROJECT_NAME}_human
  src/Human.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_human
  ${catkin_LIBRARIES}
)

# # Add cmake target dependencies of the library
# # as an example, code may need to be generated before libraries
# # either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_human ${PROJECT_NAME}_generate_messages_cpp)

# # Declare a C++ executable
# # With catkin_make all packages are built within a single CMake context
# # The recommended prefix ensures that target names across packages don't collide
add_executable(human_node src/Human_node.cpp)

# # Rename C++ executable without prefix
# # The above recommended prefix causes long target names, the following renames the
# # target back to the shorter version for ease of user use
# # e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

# # Add cmake target dependencies of the executable
# # same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(human_node ${PROJECT_NAME}_generate_messages_cpp)

# # Specify libraries to link a library or executable target against
target_link_libraries(
  human_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_human
)

add_library(move_server_lib src/move_server.cpp)
add_library(move_client_lib src/move_client.cpp)
add_library(head_server_lib src/head_server.cpp)
add_library(head_client_lib src/head_client.cpp)
add_library(arm_server_lib src/arm_server.cpp)
add_library(arm_client_lib src/arm_client.cpp)
add_library(find_color_server_lib src/find_color_server.cpp)
add_library(find_color_client_lib src/find_color_client.cpp)

add_library(robot_lib src/robot.cpp)

add_library(corridor_server_lib src/corridor_server.cpp)
add_library(scanner_server_lib src/scanner_server.cpp)

add_library(utils_lib src/utils.cpp)
add_library(map_lib src/map.cpp)
add_library(point_lib src/point.cpp)
add_library(object_lib src/object.cpp)
add_library(planning_helper_lib src/planning_helper.cpp)

add_executable(client_node src/nodes/client_node.cpp)
add_executable(move_node src/nodes/move_node.cpp)
add_executable(corridor_node src/nodes/corridor_node.cpp)
add_executable(scanner_node src/nodes/scanner_node.cpp)
add_executable(robot_node src/nodes/robot_node.cpp)
add_executable(head_node src/nodes/head_node.cpp)
add_executable(arm_node src/nodes/arm_node.cpp)
add_executable(find_color_node src/nodes/find_color_node.cpp)

# # LIBRARIES ##
target_link_libraries(
  move_client_lib
  utils_lib
  ${catkin_LIBRARIES}
)

target_link_libraries(
  arm_server_lib
  utils_lib
  planning_helper_lib
  ${catkin_LIBRARIES}
)

target_link_libraries(
  find_color_client_lib
  head_client_lib
  ${catkin_LIBRARIES}
)

target_link_libraries(
  corridor_server_lib
  point_lib
  ${catkin_LIBRARIES}
)

target_link_libraries(
  object_lib
  utils_lib
  ${catkin_LIBRARIES}
)

target_link_libraries(
  planning_helper_lib
  utils_lib
  object_lib
  ${catkin_LIBRARIES}
)

target_link_libraries(
  robot_lib
  move_client_lib
  head_client_lib
  arm_client_lib
  planning_helper_lib
  find_color_client_lib
  utils_lib
  ${catkin_LIBRARIES}
)

# # EXECUTABLE ##
target_link_libraries(
  client_node
  move_client_lib
  map_lib
  ${catkin_LIBRARIES}
)

target_link_libraries(
  move_node
  move_server_lib
  ${catkin_LIBRARIES}
)

target_link_libraries(
  corridor_node
  corridor_server_lib
  ${catkin_LIBRARIES}
)

target_link_libraries(
  scanner_node
  scanner_server_lib
  point_lib
  ${catkin_LIBRARIES}
)

target_link_libraries(
  robot_node
  robot_lib
  object_lib
  ${catkin_LIBRARIES}
)

target_link_libraries(
  head_node
  head_server_lib
  ${catkin_LIBRARIES}
)

target_link_libraries(
  arm_node
  arm_server_lib
  ${catkin_LIBRARIES}
)

target_link_libraries(
  find_color_node
  find_color_server_lib
  ${catkin_LIBRARIES}
)

add_dependencies(move_client_lib tiago_iaslab_simulation_generate_messages_cpp)
add_dependencies(move_server_lib tiago_iaslab_simulation_generate_messages_cpp)
add_dependencies(head_client_lib tiago_iaslab_simulation_generate_messages_cpp)
add_dependencies(head_server_lib tiago_iaslab_simulation_generate_messages_cpp)
add_dependencies(arm_client_lib tiago_iaslab_simulation_generate_messages_cpp)
add_dependencies(arm_server_lib tiago_iaslab_simulation_generate_messages_cpp)
add_dependencies(find_color_server_lib tiago_iaslab_simulation_generate_messages_cpp)
add_dependencies(find_color_client_lib tiago_iaslab_simulation_generate_messages_cpp)
add_dependencies(scanner_server_lib tiago_iaslab_simulation_generate_messages_cpp)

add_dependencies(client_node ${tiago_iaslab_simulation_EXPORTED_TARGETS})
add_dependencies(move_node ${tiago_iaslab_simulation_EXPORTED_TARGETS})
add_dependencies(corridor_node ${tiago_iaslab_simulation_EXPORTED_TARGETS})
add_dependencies(scanner_node ${tiago_iaslab_simulation_EXPORTED_TARGETS})
add_dependencies(robot_node ${tiago_iaslab_simulation_EXPORTED_TARGETS})
add_dependencies(head_node ${tiago_iaslab_simulation_EXPORTED_TARGETS})
add_dependencies(arm_node ${tiago_iaslab_simulation_EXPORTED_TARGETS})
add_dependencies(find_color_node ${tiago_iaslab_simulation_EXPORTED_TARGETS})
