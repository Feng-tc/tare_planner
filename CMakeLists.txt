cmake_minimum_required(VERSION 3.0.2)
project(tare_planner)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  tf2
  tf2_ros
  pcl_ros
  pcl_conversions
  message_filters
  cv_bridge
  message_generation
  building_generator_interfaces
)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

add_message_files(
  FILES
  ObjectNode.msg
  ObjectNodeList.msg
  RoomNode.msg
  RoomNodeList.msg
  RoomType.msg
  ViewpointRep.msg
  DetectionResult.msg
  RoomEarlyStop1.msg
  VlmAnswer.msg
  ObjectType.msg
  NavigationQuery.msg
  TargetObjectInstruction.msg
  TargetObject.msg
  TargetObjectWithSpatial.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
    lidar_model graph misc_utils pointcloud_utils tsp_solver viewpoint
    rolling_grid viewpoint_manager local_coverage_planner grid_world
    pointcloud_manager keypose_graph rolling_occupancy_grid planning_env
    exploration_path tare_visualizer representation sensor_coverage_planner_ground
  CATKIN_DEPENDS
    roscpp std_msgs nav_msgs geometry_msgs sensor_msgs visualization_msgs
    tf2 tf2_ros pcl_ros pcl_conversions message_filters cv_bridge message_runtime
    building_generator_interfaces
  DEPENDS PCL OpenCV
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  or-tools/include
)

link_directories(or-tools/lib)

set(TARE_CATKIN_LIBS ${catkin_LIBRARIES})
set(TARE_PCL_LIBS ${PCL_LIBRARIES})
set(TARE_ORTOOLS ${CMAKE_CURRENT_SOURCE_DIR}/or-tools/lib/libortools.so)

macro(tare_add_lib name src)
  add_library(${name} ${src})
  add_dependencies(${name} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${name} ${TARE_CATKIN_LIBS})
endmacro()

tare_add_lib(lidar_model src/lidar_model/lidar_model.cpp)
tare_add_lib(graph src/graph/graph.cpp)
tare_add_lib(misc_utils src/utils/misc_utils.cpp)
target_link_libraries(misc_utils ${TARE_PCL_LIBS} ${OpenCV_LIBRARIES})
tare_add_lib(pointcloud_utils src/utils/pointcloud_utils.cpp)
target_link_libraries(pointcloud_utils ${TARE_PCL_LIBS} misc_utils)
tare_add_lib(tsp_solver src/tsp_solver/tsp_solver.cpp)
target_link_libraries(tsp_solver ${TARE_ORTOOLS})
tare_add_lib(viewpoint src/viewpoint/viewpoint.cpp)
target_link_libraries(viewpoint lidar_model)
tare_add_lib(rolling_grid src/rolling_grid/rolling_grid.cpp)
target_link_libraries(rolling_grid misc_utils pointcloud_utils)
add_library(viewpoint_manager STATIC src/viewpoint_manager/viewpoint_manager.cpp)
add_dependencies(viewpoint_manager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(viewpoint_manager ${TARE_CATKIN_LIBS} ${OpenCV_LIBRARIES} rolling_grid viewpoint grid_world)
tare_add_lib(local_coverage_planner src/local_coverage_planner/local_coverage_planner.cpp)
target_link_libraries(local_coverage_planner viewpoint_manager planning_env)
add_library(grid_world STATIC src/grid_world/grid_world.cpp)
add_dependencies(grid_world ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(grid_world ${TARE_CATKIN_LIBS} ${OpenCV_LIBRARIES} viewpoint_manager tsp_solver keypose_graph exploration_path misc_utils)
tare_add_lib(pointcloud_manager src/pointcloud_manager/pointcloud_manager.cpp)
target_link_libraries(pointcloud_manager misc_utils)
add_library(keypose_graph STATIC src/keypose_graph/keypose_graph.cpp)
add_dependencies(keypose_graph ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(keypose_graph ${TARE_CATKIN_LIBS} ${TARE_PCL_LIBS} viewpoint_manager)
tare_add_lib(rolling_occupancy_grid src/rolling_occupancy_grid/rolling_occupancy_grid.cpp)
target_link_libraries(rolling_occupancy_grid misc_utils rolling_grid)
tare_add_lib(planning_env src/planning_env/planning_env.cpp)
target_link_libraries(planning_env ${TARE_PCL_LIBS} rolling_occupancy_grid pointcloud_manager lidar_model viewpoint_manager grid_world)
tare_add_lib(exploration_path src/exploration_path/exploration_path.cpp)
target_link_libraries(exploration_path ${TARE_PCL_LIBS})
tare_add_lib(tare_visualizer src/tare_visualizer/tare_visualizer.cpp)
target_link_libraries(tare_visualizer ${TARE_PCL_LIBS} grid_world misc_utils planning_env)
tare_add_lib(representation src/representation/representation.cpp)
target_link_libraries(representation ${OpenCV_LIBRARIES} misc_utils pointcloud_utils)
tare_add_lib(sensor_coverage_planner_ground src/sensor_coverage_planner/sensor_coverage_planner_ground.cpp)
target_link_libraries(sensor_coverage_planner_ground ${OpenCV_LIBRARIES} graph planning_env keypose_graph viewpoint_manager pointcloud_manager grid_world local_coverage_planner tare_visualizer misc_utils representation)

add_executable(navigationBoundary src/navigation_boundary_publisher/navigationBoundary.cpp)
add_dependencies(navigationBoundary ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(navigationBoundary ${TARE_CATKIN_LIBS} ${TARE_PCL_LIBS})

add_executable(tare_planner_node src/tare_planner_node/tare_planner_node.cpp)
add_dependencies(tare_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tare_planner_node sensor_coverage_planner_ground)

add_executable(room_segmentation src/room_segmentation/room_segmentation.cpp)
add_dependencies(room_segmentation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(room_segmentation ${TARE_CATKIN_LIBS} ${OpenCV_LIBRARIES} pointcloud_utils representation misc_utils)

install(TARGETS
  lidar_model graph misc_utils pointcloud_utils tsp_solver viewpoint rolling_grid
  viewpoint_manager local_coverage_planner grid_world pointcloud_manager keypose_graph
  rolling_occupancy_grid planning_env exploration_path tare_visualizer representation
  sensor_coverage_planner_ground navigationBoundary tare_planner_node room_segmentation
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY launch/ config/ rviz/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
