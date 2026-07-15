/**
 * @file sensor_coverage_planner_ground.cpp
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that does the job of exploration
 * @version 0.1
 * @date 2020-06-03
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "sensor_coverage_planner/sensor_coverage_planner_ground.h"
#include "graph/graph.h"
#include <limits>
#include <memory>
#include <unordered_map>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
using json = nlohmann::json;



namespace sensor_coverage_planner_3d_ns {
void SensorCoveragePlanner3D::ReadParameters() {
  private_nh_.param<std::string>("sub_start_exploration_topic_", sub_start_exploration_topic_, "/exploration_start");
  private_nh_.param<std::string>("sub_state_estimation_topic_", sub_state_estimation_topic_, "/Odometry_gazebo");
  private_nh_.param<std::string>("sub_registered_scan_topic_", sub_registered_scan_topic_, "/livox/Pointcloud2");
  private_nh_.param<std::string>("sub_terrain_map_topic_", sub_terrain_map_topic_, "/terrain_map");
  private_nh_.param<std::string>("sub_terrain_map_ext_topic_", sub_terrain_map_ext_topic_, "/terrain_map_ext");
  private_nh_.param<std::string>("sub_coverage_boundary_topic_", sub_coverage_boundary_topic_, "/coverage_boundary");
  private_nh_.param<std::string>("sub_viewpoint_boundary_topic_", sub_viewpoint_boundary_topic_, "/navigation_boundary");
  private_nh_.param<std::string>("sub_viewpoint_room_boundary_topic_", sub_viewpoint_room_boundary_topic_, "/current_room_boundary");
  private_nh_.param<std::string>("sub_nogo_boundary_topic_", sub_nogo_boundary_topic_, "/nogo_boundary");
  private_nh_.param<std::string>("sub_joystick_topic_", sub_joystick_topic_, "/joy");
  private_nh_.param<std::string>("sub_reset_waypoint_topic_", sub_reset_waypoint_topic_, "/reset_waypoint");
  private_nh_.param<std::string>("pub_exploration_finish_topic_", pub_exploration_finish_topic_, "exploration_finish");
  private_nh_.param<std::string>("pub_runtime_breakdown_topic_", pub_runtime_breakdown_topic_, "runtime_breakdown");
  private_nh_.param<std::string>("pub_runtime_topic_", pub_runtime_topic_, "/runtime");
  private_nh_.param<std::string>("pub_waypoint_topic_", pub_waypoint_topic_, "/way_point");
  private_nh_.param<std::string>("pub_momentum_activation_count_topic_", pub_momentum_activation_count_topic_, "momentum_activation_count");

  private_nh_.param("kAutoStart", kAutoStart, false);
  private_nh_.param("kRushHome", kRushHome, false);
  private_nh_.param("kUseTerrainHeight", kUseTerrainHeight, true);
  private_nh_.param("kCheckTerrainCollision", kCheckTerrainCollision, true);
  private_nh_.param("kExtendWayPoint", kExtendWayPoint, true);
  private_nh_.param("kUseLineOfSightLookAheadPoint", kUseLineOfSightLookAheadPoint, true);
  private_nh_.param("kNoExplorationReturnHome", kNoExplorationReturnHome, true);
  private_nh_.param("kUseMomentum", kUseMomentum, false);
  private_nh_.param("kUseVlm", kUseVlm, false);
  private_nh_.param<std::string>("kTargetObject", kTargetObject, "");

  private_nh_.param("kKeyposeCloudDwzFilterLeafSize", kKeyposeCloudDwzFilterLeafSize, 0.2);
  private_nh_.param("kRushHomeDist", kRushHomeDist, 10.0);
  private_nh_.param("kRushRoomDist_1", kRushRoomDist_1, 4.0);
  private_nh_.param("kRushRoomDist_2", kRushRoomDist_2, 1.8);
  private_nh_.param("kAtHomeDistThreshold", kAtHomeDistThreshold, 0.5);
  private_nh_.param("kTerrainCollisionThreshold", kTerrainCollisionThreshold, 0.5);
  private_nh_.param("kLookAheadDistance", kLookAheadDistance, 5.0);
  private_nh_.param("kExtendWayPointDistanceBig", kExtendWayPointDistanceBig, 8.0);
  private_nh_.param("kExtendWayPointDistanceSmall", kExtendWayPointDistanceSmall, 3.0);

  private_nh_.param("kWaypointPlanningScanInterval", kWaypointPlanningScanInterval, 5);
  if (kWaypointPlanningScanInterval < 1) {
    ROS_WARN("kWaypointPlanningScanInterval must be >= 1, resetting to 1");
    kWaypointPlanningScanInterval = 1;
  }

  private_nh_.param("kDirectionChangeCounterThr", kDirectionChangeCounterThr, 4);
  private_nh_.param("kDirectionNoChangeCounterThr", kDirectionNoChangeCounterThr, 5);
  private_nh_.param("kResetWaypointJoystickAxesID", kResetWaypointJoystickAxesID, 0);
  private_nh_.param("rep_threshold_", rep_threshold_, 0.1);
  private_nh_.param("kRepSensorRange", rep_sensor_range, 5.0);

  private_nh_.param("room_resolution", room_resolution_, 0.1f);
  private_nh_.param("rolling_occupancy_grid/resolution_x", occupancy_grid_resolution_, 0.3f);
  int room_x = 200, room_y = 200, room_z = 50;
  private_nh_.param("room_x", room_x, 200);
  private_nh_.param("room_y", room_y, 200);
  private_nh_.param("room_z", room_z, 50);
  room_voxel_dimension_.x() = room_x;
  room_voxel_dimension_.y() = room_y;
  room_voxel_dimension_.z() = room_z;
}

// void PlannerData::Initialize(ros::NodeHandle* node_)
void SensorCoveragePlanner3D::InitializeData() {
  keypose_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>>(
          &nh_, "keypose_cloud", kWorldFrameID);
  registered_scan_stack_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZ>>(
          &nh_, "registered_scan_stack", kWorldFrameID);
  registered_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
          &nh_, "registered_cloud", kWorldFrameID);
  large_terrain_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
          &nh_, "terrain_cloud_large", kWorldFrameID);
  terrain_collision_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
          &nh_, "terrain_collision_cloud", kWorldFrameID);
  terrain_ext_collision_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
          &nh_, "terrain_ext_collision_cloud", kWorldFrameID);
  viewpoint_vis_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
          &nh_, "viewpoint_vis_cloud", kWorldFrameID);
  grid_world_vis_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
          &nh_, "grid_world_vis_cloud", kWorldFrameID);
  exploration_path_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
          &nh_, "bspline_path_cloud", kWorldFrameID);

  selected_viewpoint_vis_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
          &nh_, "selected_viewpoint_vis_cloud", kWorldFrameID);
  exploring_cell_vis_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
          &nh_, "exploring_cell_vis_cloud", kWorldFrameID);
  collision_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
          &nh_, "collision_cloud", kWorldFrameID);
  lookahead_point_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
          &nh_, "lookahead_point_cloud", kWorldFrameID);
  keypose_graph_vis_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
          &nh_, "keypose_graph_cloud", kWorldFrameID);
  viewpoint_in_collision_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
          &nh_, "viewpoint_in_collision_cloud_", kWorldFrameID);
  point_cloud_manager_neighbor_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
          &nh_, "pointcloud_manager_cloud", kWorldFrameID);
  reordered_global_subspace_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
          &nh_, "reordered_global_subspace_cloud", kWorldFrameID);
  freespace_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
          &nh_, "freespace_cloud", kWorldFrameID);

  viewpoint_manager_ = std::make_shared<viewpoint_manager_ns::ViewPointManager>(
      &private_nh_);
  keypose_graph_ =
      std::make_shared<keypose_graph_ns::KeyposeGraph>(&private_nh_);
  planning_env_ =
      std::make_shared<planning_env_ns::PlanningEnv>(&private_nh_);
  grid_world_ = std::make_shared<grid_world_ns::GridWorld>(&private_nh_);
  grid_world_->SetUseKeyposeGraph(true);
  local_coverage_planner_ =
      std::make_shared<local_coverage_planner_ns::LocalCoveragePlanner>(
          &private_nh_);
  local_coverage_planner_->SetViewPointManager(viewpoint_manager_);

  visualizer_ =
      std::make_shared<tare_visualizer_ns::TAREVisualizer>(&private_nh_);

  initial_position_.x() = 0.0;
  initial_position_.y() = 0.0;
  initial_position_.z() = 0.0;

  cur_keypose_node_ind_ = 0;

  keypose_graph_node_marker_ = std::make_shared<misc_utils_ns::Marker>(
      &nh_, "keypose_graph_node_marker", kWorldFrameID);
  keypose_graph_node_marker_->SetType(visualization_msgs::Marker::POINTS);
  keypose_graph_node_marker_->SetScale(0.4, 0.4, 0.1);
  keypose_graph_node_marker_->SetColorRGBA(1.0, 0.0, 0.0, 1.0);
  keypose_graph_edge_marker_ = std::make_shared<misc_utils_ns::Marker>(
      &nh_, "keypose_graph_edge_marker", kWorldFrameID);
  keypose_graph_edge_marker_->SetType(
      visualization_msgs::Marker::LINE_LIST);
  keypose_graph_edge_marker_->SetScale(0.05, 0.0, 0.0);
  keypose_graph_edge_marker_->SetColorRGBA(1.0, 1.0, 0.0, 0.9);

  nogo_boundary_marker_ = std::make_shared<misc_utils_ns::Marker>(
      &nh_, "nogo_boundary_marker", kWorldFrameID);
  nogo_boundary_marker_->SetType(visualization_msgs::Marker::LINE_LIST);
  nogo_boundary_marker_->SetScale(0.05, 0.0, 0.0);
  nogo_boundary_marker_->SetColorRGBA(1.0, 0.0, 0.0, 0.8);

  grid_world_marker_ = std::make_shared<misc_utils_ns::Marker>(
      &nh_, "grid_world_marker", kWorldFrameID);
  grid_world_marker_->SetType(visualization_msgs::Marker::CUBE_LIST);
  grid_world_marker_->SetScale(1.0, 1.0, 1.0);
  grid_world_marker_->SetColorRGBA(1.0, 0.0, 0.0, 0.8);

  robot_yaw_ = 0.0;
  lookahead_point_direction_ = Eigen::Vector3d(1.0, 0.0, 0.0);
  moving_direction_ = Eigen::Vector3d(1.0, 0.0, 0.0);
  moving_forward_ = true;

  Eigen::Vector3d viewpoint_resolution = viewpoint_manager_->GetResolution();
  double add_non_keypose_node_min_dist =
      std::min(viewpoint_resolution.x(), viewpoint_resolution.y()) / 2;
  keypose_graph_->SetAddNonKeyposeNodeMinDist() = add_non_keypose_node_min_dist;

  robot_position_.x = 0;
  robot_position_.y = 0;
  robot_position_.z = 0;

  last_robot_position_ = robot_position_;

  // ========== VLM-Related Initialization ==========
  
  // Representation core
  representation_ = std::make_shared<representation_ns::Representation>(&nh_, kWorldFrameID);

  // Viewpoint representation initialization
  double resolution = occupancy_grid_resolution_;
  if (resolution <= 0.0) {
    resolution = 0.3;
    occupancy_grid_resolution_ = static_cast<float>(resolution);
  }
  rep_threshold_voxel_num_ = int(rep_threshold_ * (2.0 / 3.0 * M_PI * std::pow(rep_sensor_range, 3)) /
                                 (resolution * resolution * resolution));
  add_viewpoint_rep_ = false;
  curr_viewpoint_rep_node_ind = 0;
  
  viewpoint_reps_ = std::vector<representation_ns::ViewPointRep>();
  previous_obs_voxel_inds_ = std::vector<int>();
  current_obs_voxel_inds_ = std::vector<int>();
  
  viewpoint_rep_vis_cloud_ = std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZ>>(
      &nh_, "viewpoint_rep_vis_cloud", kWorldFrameID);
  covered_points_all_ = std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
      &nh_, "viewpoint_rep_covered_points", kWorldFrameID);
  viewpoint_rep_msg_ = tare_planner::ViewpointRep();

  // Door and room boundary initialization
  door_cloud_ = pcl::PointCloud<pcl::PointXYZRGBL>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBL>());
  door_cloud_final_ = pcl::PointCloud<pcl::PointXYZRGBL>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBL>());
  door_cloud_vis_ = std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZRGBL>>(
      &nh_, "door_cloud_vis", kWorldFrameID);
  door_cloud_in_range_ = std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZLNormal>>(
      &nh_, "door_cloud_in_range", kWorldFrameID);
  door_position_ = Eigen::Vector3d(-10000.0, -10000.0, -10000.0);
  door_normal_ = Eigen::Vector3d(0.0, 0.0, 0.0);

  // Room state flags initialization
  transit_across_room_ = false;
  at_room_ = false;
  near_room_1_ = false;
  near_room_2_ = false;
  enter_wrong_room_ = false;
  asked_in_advance_ = false;
  has_candidate_room_position_ = false;

  // Room data structures initialization (room_voxel_dimension_ loaded in ReadParameters)
  adjacency_matrix = Eigen::MatrixXi::Zero(200, 200);
  shift_ = Eigen::Vector3f(room_voxel_dimension_.x() / 2.0,
                           room_voxel_dimension_.y() / 2.0,
                           room_voxel_dimension_.z() / 2.0);
  room_mask_ = cv::Mat::zeros(room_voxel_dimension_.x(), room_voxel_dimension_.y(), CV_32S);
  room_mask_old_ = room_mask_.clone();

  // Room IDs and positions initialization
  current_room_id_ = -1;
  target_room_id_ = -1;
  start_room_id_ = -1;
  end_room_id_ = -1;
  prev_room_id_ = -1;
  previous_room_id_ = -1;
  robot_position_old_ = robot_position_;
  goal_position_.x = -10000.0;
  goal_position_.y = -10000.0;
  goal_position_.z = -10000.0;
  candidate_room_position_ = geometry_msgs::Point();
  candidate_room_id_ = -1;

  // Room counters and parameters initialization
  room_guide_counter_ = 0;
  room_id_change_counter_ = 0;
  room_navigation_query_counter_ = 0;
  stayed_in_room_counter_ = 0;
  room_finished_counter_ = 0;
  // room_resolution_, kRushRoomDist_* already loaded in ReadParameters()

  // Target object tracking initialization
  found_object_ = false;
  ask_found_object_ = false;
  found_object_id_ = -1;
  found_object_room_id_ = -1;
  found_object_distance_ = 10000.0;
  found_object_position_ = geometry_msgs::Point();
  target_object_ = "";

  // Anchor object tracking initialization
  found_anchor_object_ = false;
  ask_found_anchor_object_ = false;
  found_anchor_object_id_ = -1;
  found_anchor_object_room_id_ = -1;
  found_anchor_object_distance_ = 10000.0;
  found_anchor_object_position_ = geometry_msgs::Point();
  found_anchor_object_viewpoint_positions_.clear();
  found_anchor_object_viewpoint_positions_visited_.clear();
  anchor_object_ = "";

  // Object detection parameters initialization
  last_object_update_time_ = ros::Time::now();
  object_ids_to_remove_ = std::vector<int>();
  obj_score_ = 0.0;
  considered_object_ids_ = std::set<int>();

  // Search and navigation conditions initialization
  room_condition_ = "";
  spatial_condition_ = "";
  attribute_condition_ = "";

  // Camera and sensor data initialization
  camera_image_ = cv::Mat::zeros(640, 1920, CV_8UC3);
  freespace_cloud_ = std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
      &nh_, "freespace_cloud", kWorldFrameID);

  // Odometry stack initialization
  odomLastIDPointer = -1;
  odomFrontIDPointer = 0;
  odomTime = 0.0;
  imageTime = 0.0;
  odomX = 0.0;
  odomY = 0.0;
  odomZ = 0.0;
  PI = 3.14159265358979323846;

  // Timing initialization
  last_target_object_instruction_time_ = ros::Time::now();

  // Miscellaneous flags initialization
  dynamic_environment_ = false;
  tmp_flag_ = false;
}

SensorCoveragePlanner3D::SensorCoveragePlanner3D(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
    : nh_(nh), private_nh_(private_nh), keypose_cloud_update_(false),
      initialized_(false), lookahead_point_update_(false), relocation_(false),
      start_exploration_(false), exploration_finished_(false),
      near_home_(false), at_home_(false), stopped_(false),
      test_point_update_(false), viewpoint_ind_update_(false), step_(false),
      use_momentum_(false), lookahead_point_in_line_of_sight_(true),
      reset_waypoint_(false), registered_cloud_count_(0), keypose_count_(0),
      direction_change_count_(0), direction_no_change_count_(0),
      momentum_activation_count_(0), reset_waypoint_joystick_axis_value_(-1.0),
      add_viewpoint_rep_(false), at_room_(false), near_room_1_(false), near_room_2_(false)
{
  std::cout << "finished constructor" << std::endl;
}

bool SensorCoveragePlanner3D::initialize() {
  ReadParameters();
  if (!kTargetObject.empty()) {
    target_object_ = kTargetObject;
    ROS_INFO("Target object set from parameter: %s (kUseVlm=%d)", target_object_.c_str(), kUseVlm);
  }
  InitializeData();

  keypose_graph_->SetAllowVerticalEdge(false);

  lidar_model_ns::LiDARModel::setCloudDWZResol(
      planning_env_->GetPlannerCloudResolution());

  execution_timer_ = nh_.createTimer(ros::Duration(1.0), &SensorCoveragePlanner3D::executeTimer, this);

  exploration_start_sub_ = nh_.subscribe<std_msgs::Bool>(sub_start_exploration_topic_, 5, &SensorCoveragePlanner3D::ExplorationStartCallback, this);
  registered_scan_sub_ =
      nh_.subscribe<sensor_msgs::PointCloud2>(sub_registered_scan_topic_, 5, &SensorCoveragePlanner3D::RegisteredScanCallback, this);
  terrain_map_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(sub_terrain_map_topic_, 5, &SensorCoveragePlanner3D::TerrainMapCallback, this);
  terrain_map_ext_sub_ =
      nh_.subscribe<sensor_msgs::PointCloud2>(sub_terrain_map_ext_topic_, 5, &SensorCoveragePlanner3D::TerrainMapExtCallback, this);
  state_estimation_sub_ = nh_.subscribe<nav_msgs::Odometry>(sub_state_estimation_topic_, 5, &SensorCoveragePlanner3D::StateEstimationCallback, this);
  coverage_boundary_sub_ =
      nh_.subscribe<geometry_msgs::PolygonStamped>(sub_coverage_boundary_topic_, 5, &SensorCoveragePlanner3D::CoverageBoundaryCallback, this);
  viewpoint_boundary_sub_ =
      nh_.subscribe<geometry_msgs::PolygonStamped>(sub_viewpoint_boundary_topic_, 5, &SensorCoveragePlanner3D::ViewPointBoundaryCallback, this);
  nogo_boundary_sub_ =
      nh_.subscribe<geometry_msgs::PolygonStamped>(sub_nogo_boundary_topic_, 5, &SensorCoveragePlanner3D::NogoBoundaryCallback, this);
  joystick_sub_ = nh_.subscribe<sensor_msgs::Joy>(sub_joystick_topic_, 5, &SensorCoveragePlanner3D::JoystickCallback, this);
  reset_waypoint_sub_ = nh_.subscribe<std_msgs::Empty>(sub_reset_waypoint_topic_, 1, &SensorCoveragePlanner3D::ResetWaypointCallback, this);
  object_node_list_sub_ = nh_.subscribe<tare_planner::ObjectNodeList>("/object_nodes_list", 20, &SensorCoveragePlanner3D::ObjectNodeListCallback, this);
  door_cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/door_cloud", 5, &SensorCoveragePlanner3D::DoorCloudCallback, this);
  room_node_list_sub_ = nh_.subscribe<tare_planner::RoomNodeList>("/room_nodes_list", 5, &SensorCoveragePlanner3D::RoomNodeListCallback, this);
  goal_point_sub_ = nh_.subscribe<geometry_msgs::PointStamped>("/goal_point", 5, &SensorCoveragePlanner3D::GoalPointCallback, this);
  room_mask_sub_ = nh_.subscribe<sensor_msgs::Image>("/room_mask", 5, &SensorCoveragePlanner3D::RoomMaskCallback, this);
  camera_image_sub_ = nh_.subscribe<sensor_msgs::Image>("/camera/image_raw", 5, &SensorCoveragePlanner3D::CameraImageCallback, this);
  room_type_sub_ = nh_.subscribe<tare_planner::RoomType>("/room_type_answer", 10, &SensorCoveragePlanner3D::RoomTypeCallback, this);
  keyboard_input_sub_ = nh_.subscribe<std_msgs::String>("/keyboard_input", 5, &SensorCoveragePlanner3D::KeyboardInputCallback, this);
  target_object_instruction_sub_ = nh_.subscribe<tare_planner::TargetObjectInstruction>("/target_object_instruction", 5, &SensorCoveragePlanner3D::TargetObjectInstructionCallback, this);
  target_object_sub_ = nh_.subscribe<tare_planner::TargetObject>("/target_object_answer", 5, &SensorCoveragePlanner3D::TargetObjectCallback, this);
  anchor_object_sub_ = nh_.subscribe<tare_planner::TargetObject>("/anchor_object_answer", 5, &SensorCoveragePlanner3D::AnchorObjectCallback, this);

  global_path_full_publisher_ =
      nh_.advertise<nav_msgs::Path>("global_path_full", 1);
  global_path_publisher_ =
      nh_.advertise<nav_msgs::Path>("global_path", 1);
  old_global_path_publisher_ =
      nh_.advertise<nav_msgs::Path>("old_global_path", 1);
  to_nearest_global_subspace_path_publisher_ =
      nh_.advertise<nav_msgs::Path>(
          "to_nearest_global_subspace_path", 1);
  local_tsp_path_publisher_ =
      nh_.advertise<nav_msgs::Path>("local_path", 1);
  exploration_path_publisher_ =
      nh_.advertise<nav_msgs::Path>("exploration_path", 1);
  waypoint_pub_ = nh_.advertise<geometry_msgs::PointStamped>(
      pub_waypoint_topic_, 2);
  exploration_finish_pub_ = nh_.advertise<std_msgs::Bool>(
      pub_exploration_finish_topic_, 2);
  runtime_breakdown_pub_ =
      nh_.advertise<std_msgs::Int32MultiArray>(
          pub_runtime_breakdown_topic_, 2);
  runtime_pub_ =
      nh_.advertise<std_msgs::Float32>(pub_runtime_topic_, 2);
  momentum_activation_count_pub_ = nh_.advertise<std_msgs::Int32>(
      pub_momentum_activation_count_topic_, 2);
  pointcloud_manager_neighbor_cells_origin_pub_ =
      nh_.advertise<geometry_msgs::PointStamped>(
          "pointcloud_manager_neighbor_cells_origin", 1);
  viewpoint_rep_pub_ =
      nh_.advertise<tare_planner::ViewpointRep>("viewpoint_rep_header", 5);
  object_visibility_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
    "object_visibility_connections", 1);
  viewpoint_visibility_pub_ = nh_.advertise<std_msgs::String>(
      "viewpoint_object_visibility", 1);
  door_position_pub_ =
      nh_.advertise<geometry_msgs::PointStamped>(
          "/door_position", 1);
  door_normal_pub_ = nh_.advertise<visualization_msgs::Marker>(
      "/door_normal", 1);
  room_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
      "/room_cloud", 1);
  room_type_pub_ = nh_.advertise<tare_planner::RoomType>(
      "/room_type_query", 10);
  room_type_vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "/room_type_vis", 5);
  viewpoint_room_id_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "viewpoint_room_ids", 1);
  object_node_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "/object_node_markers", 1);
  chosen_room_boundary_pub_ = nh_.advertise<visualization_msgs::Marker>(
      "/chosen_room_boundary", 1);
  target_object_pub_ = nh_.advertise<tare_planner::TargetObject>(
      "/target_object_query", 5);
  anchor_object_pub_ = nh_.advertise<tare_planner::TargetObject>(
      "/anchor_object_query", 5);
  target_object_spatial_pub_ = nh_.advertise<tare_planner::TargetObjectWithSpatial>(
      "/target_object_spatial_query", 5);
  room_anchor_point_pub_ = nh_.advertise<geometry_msgs::PointStamped>(
      "/room_anchor_point", 5);

  PrintExplorationStatus("Exploration Started", false);
  return true;
}

void SensorCoveragePlanner3D::ExplorationStartCallback(
    const std_msgs::Bool::ConstPtr& start_msg) {
  if (start_msg->data) {
    start_exploration_ = true;
  }
}

void SensorCoveragePlanner3D::StateEstimationCallback(
    const nav_msgs::Odometry::ConstPtr& state_estimation_msg) {
  robot_position_ = state_estimation_msg->pose.pose.position;
  // Todo: use a boolean
  if (std::abs(initial_position_.x()) < 0.01 &&
      std::abs(initial_position_.y()) < 0.01 &&
      std::abs(initial_position_.z()) < 0.01) {
    initial_position_.x() = robot_position_.x;
    initial_position_.y() = robot_position_.y;
    initial_position_.z() = robot_position_.z;
  }
  double roll, pitch, yaw;
  geometry_msgs::Quaternion geo_quat =
      state_estimation_msg->pose.pose.orientation;
  tf2::Matrix3x3(
      tf2::Quaternion(geo_quat.x, geo_quat.y, geo_quat.z, geo_quat.w))
      .getRPY(roll, pitch, yaw);

  robot_yaw_ = yaw;

  if (state_estimation_msg->twist.twist.linear.x > 0.4) {
    moving_forward_ = true;
  } else if (state_estimation_msg->twist.twist.linear.x < -0.4) {
    moving_forward_ = false;
  }

  // Get the timestamp from the state estimation message(for case of ros bag)
  viewpoint_rep_msg_.header.stamp = state_estimation_msg->header.stamp;
  viewpoint_rep_msg_.header.frame_id = kWorldFrameID;
  // initialized_ = true;

  odomTime = ros::Time(state_estimation_msg->header.stamp).toSec();
  odomX = state_estimation_msg->pose.pose.position.x;
  odomY = state_estimation_msg->pose.pose.position.y;
  odomZ = state_estimation_msg->pose.pose.position.z;

  odomLastIDPointer = (odomLastIDPointer + 1) % 400;
  odomTimeStack[odomLastIDPointer] = odomTime;
  lidarXStack[odomLastIDPointer] = odomX;
  lidarYStack[odomLastIDPointer] = odomY;
  lidarZStack[odomLastIDPointer] = odomZ;
  lidarRollStack[odomLastIDPointer] = roll;
  lidarPitchStack[odomLastIDPointer] = pitch;
  lidarYawStack[odomLastIDPointer] = yaw;
}

void SensorCoveragePlanner3D::CameraImageCallback(
    const sensor_msgs::Image::ConstPtr& camera_image_msg)
{
  if (!initialized_)
  {
    return;
  }
  if (camera_image_msg->data.empty())
  {
    ROS_ERROR("Camera image data is empty");
    return;
  }
  // 转换成 OpenCV 格式
  camera_image_ = cv_bridge::toCvCopy(camera_image_msg, "bgr8")->image;
  imageTime = ros::Time(camera_image_msg->header.stamp).toSec();
}

void SensorCoveragePlanner3D::RegisteredScanCallback(
    const sensor_msgs::PointCloud2::ConstPtr& registered_scan_msg) {
  if (!initialized_) {
    return;
  }

  registered_cloud_count_ = (registered_cloud_count_ + 1) % kWaypointPlanningScanInterval;

  pcl::PointCloud<pcl::PointXYZ>::Ptr registered_scan_tmp(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*registered_scan_msg, *registered_scan_tmp);
  if (registered_scan_tmp->points.empty()) {
    return;
  }
  *(registered_scan_stack_->cloud_) += *(registered_scan_tmp);
  pointcloud_downsizer_.Downsize(
      registered_scan_tmp, kKeyposeCloudDwzFilterLeafSize,
      kKeyposeCloudDwzFilterLeafSize, kKeyposeCloudDwzFilterLeafSize);
  registered_cloud_->cloud_->clear();
  pcl::copyPointCloud(*registered_scan_tmp, *(registered_cloud_->cloud_));

  planning_env_->UpdateRobotPosition(robot_position_);
  planning_env_->UpdateRegisteredCloud<pcl::PointXYZI>(
      registered_cloud_->cloud_, registered_cloud_count_);

  if (registered_cloud_count_ == 0) {
    // initialized_ = true;
    keypose_.pose.pose.position = robot_position_;
    keypose_.pose.covariance[0] = keypose_count_++;
    cur_keypose_node_ind_ =
        keypose_graph_->AddKeyposeNode(keypose_, *(planning_env_));

    pointcloud_downsizer_.Downsize(
        registered_scan_stack_->cloud_, kKeyposeCloudDwzFilterLeafSize,
        kKeyposeCloudDwzFilterLeafSize, kKeyposeCloudDwzFilterLeafSize);

    keypose_cloud_->cloud_->clear();
    pcl::copyPointCloud(*(registered_scan_stack_->cloud_),
                        *(keypose_cloud_->cloud_));
    // keypose_cloud_->Publish();
    registered_scan_stack_->cloud_->clear();
    keypose_cloud_update_ = true;
  }
}

void SensorCoveragePlanner3D::TerrainMapCallback(
    const sensor_msgs::PointCloud2::ConstPtr& terrain_map_msg) {
  if (kCheckTerrainCollision) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr terrain_map_tmp(
        new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg<pcl::PointXYZI>(*terrain_map_msg, *terrain_map_tmp);
    terrain_collision_cloud_->cloud_->clear();
    for (auto &point : terrain_map_tmp->points) {
      if (point.intensity > kTerrainCollisionThreshold) {
        terrain_collision_cloud_->cloud_->points.push_back(point);
      }
    }
  }
}

void SensorCoveragePlanner3D::TerrainMapExtCallback(
    const sensor_msgs::PointCloud2::ConstPtr& terrain_map_ext_msg) {
  if (kUseTerrainHeight) {
    pcl::fromROSMsg<pcl::PointXYZI>(*terrain_map_ext_msg,
                                    *(large_terrain_cloud_->cloud_));
  }
  if (kCheckTerrainCollision) {
    pcl::fromROSMsg<pcl::PointXYZI>(*terrain_map_ext_msg,
                                    *(large_terrain_cloud_->cloud_));
    terrain_ext_collision_cloud_->cloud_->clear();
    for (auto &point : large_terrain_cloud_->cloud_->points) {
      if (point.intensity > kTerrainCollisionThreshold) {
        terrain_ext_collision_cloud_->cloud_->points.push_back(point);
      }
    }
  }
}

void SensorCoveragePlanner3D::CoverageBoundaryCallback(
    const geometry_msgs::PolygonStamped::ConstPtr& polygon_msg) {
  planning_env_->UpdateCoverageBoundary((*polygon_msg).polygon);
}

void SensorCoveragePlanner3D::ViewPointBoundaryCallback(
    const geometry_msgs::PolygonStamped::ConstPtr& polygon_msg) {
  viewpoint_manager_->UpdateViewPointBoundary((*polygon_msg).polygon);
}

void SensorCoveragePlanner3D::NogoBoundaryCallback(
    const geometry_msgs::PolygonStamped::ConstPtr& polygon_msg) {
  if (polygon_msg->polygon.points.empty()) {
    return;
  }
  double polygon_id = polygon_msg->polygon.points[0].z;
  int polygon_point_size = polygon_msg->polygon.points.size();
  std::vector<geometry_msgs::Polygon> nogo_boundary;
  geometry_msgs::Polygon polygon;
  for (int i = 0; i < polygon_point_size; i++) {
    if (polygon_msg->polygon.points[i].z == polygon_id) {
      polygon.points.push_back(polygon_msg->polygon.points[i]);
    } else {
      nogo_boundary.push_back(polygon);
      polygon.points.clear();
      polygon_id = polygon_msg->polygon.points[i].z;
      polygon.points.push_back(polygon_msg->polygon.points[i]);
    }
  }
  nogo_boundary.push_back(polygon);
  viewpoint_manager_->UpdateNogoBoundary(nogo_boundary);

  geometry_msgs::Point point;
  for (int i = 0; i < nogo_boundary.size(); i++) {
    for (int j = 0; j < nogo_boundary[i].points.size() - 1; j++) {
      point.x = nogo_boundary[i].points[j].x;
      point.y = nogo_boundary[i].points[j].y;
      point.z = nogo_boundary[i].points[j].z;
      nogo_boundary_marker_->marker_.points.push_back(point);
      point.x = nogo_boundary[i].points[j + 1].x;
      point.y = nogo_boundary[i].points[j + 1].y;
      point.z = nogo_boundary[i].points[j + 1].z;
      nogo_boundary_marker_->marker_.points.push_back(point);
    }
    point.x = nogo_boundary[i].points.back().x;
    point.y = nogo_boundary[i].points.back().y;
    point.z = nogo_boundary[i].points.back().z;
    nogo_boundary_marker_->marker_.points.push_back(point);
    point.x = nogo_boundary[i].points.front().x;
    point.y = nogo_boundary[i].points.front().y;
    point.z = nogo_boundary[i].points.front().z;
    nogo_boundary_marker_->marker_.points.push_back(point);
  }
  nogo_boundary_marker_->Publish();
}

void SensorCoveragePlanner3D::JoystickCallback(
    const sensor_msgs::Joy::ConstPtr& joy_msg) {
  if (kResetWaypointJoystickAxesID >= 0 &&
      kResetWaypointJoystickAxesID < joy_msg->axes.size()) {
    if (reset_waypoint_joystick_axis_value_ > -0.1 &&
        joy_msg->axes[kResetWaypointJoystickAxesID] < -0.1) {
      reset_waypoint_ = true;

      // Set waypoint to the current robot position to stop the robot in place
      geometry_msgs::PointStamped waypoint;
      waypoint.header.frame_id = "map";
      waypoint.header.stamp = ros::Time::now();
      waypoint.point.x = robot_position_.x;
      waypoint.point.y = robot_position_.y;
      waypoint.point.z = robot_position_.z;
      waypoint_pub_.publish(waypoint);
      std::cout << "reset waypoint" << std::endl;
    }
    reset_waypoint_joystick_axis_value_ =
        joy_msg->axes[kResetWaypointJoystickAxesID];
  }
}

void SensorCoveragePlanner3D::ResetWaypointCallback(
    const std_msgs::Empty::ConstPtr& empty_msg) {
  reset_waypoint_ = true;

  // Set waypoint to the current robot position to stop the robot in place
  geometry_msgs::PointStamped waypoint;
  waypoint.header.frame_id = "map";
  waypoint.header.stamp = ros::Time::now();
  waypoint.point.x = robot_position_.x;
  waypoint.point.y = robot_position_.y;
  waypoint.point.z = robot_position_.z;
  waypoint_pub_.publish(waypoint);
  std::cout << "reset waypoint" << std::endl;
}

void SensorCoveragePlanner3D::ObjectNodeListCallback(
    const tare_planner::ObjectNodeList::ConstPtr& msg) {
    
  if (!initialized_) {
      return;
  }

  if (msg->nodes.empty()) {
      ROS_DEBUG("Received empty ObjectNodeList");
      return;
  }

  // Single timestamp check and logging for the entire batch
  ros::Time now = ros::Time::now();
  ros::Duration time_diff = now - msg->header.stamp;
  ROS_INFO("Received ObjectNodeList with %zu objects, time_diff=%.2f seconds", 
              msg->nodes.size(), time_diff.toSec());
  
  last_object_update_time_ = msg->header.stamp;
  
  // Process all objects in the batch
  int deleted_count = 0;
  int updated_count = 0;
  int skipped_count = 0;
  
  for (const auto& node : msg->nodes) {
    // Convert to ConstSharedPtr for compatibility with existing UpdateObjectNode
    auto node_ptr = boost::make_shared<tare_planner::ObjectNode>(node);
    
    // false for deleted objects, true for updated/new objects
    if (node.status == false) {
      for (auto obj_id : node.object_id) {
        // TODO: temporarily don't delete the found object
        if (obj_id == found_object_id_) {
          continue;
        }
        object_ids_to_remove_.push_back(obj_id);
        deleted_count++;
      }
      continue;
    }

    // Skip objects with empty cloud
    if (node.cloud.data.empty()) {
      skipped_count++;
      continue;
    }

    // Update representation
    representation_->UpdateObjectNode(node_ptr);
    representation_->GetLatestObjectNodeIndicesMutable().insert(node.object_id[0]);
    updated_count++;
  }
  
  ROS_DEBUG("Batch processed: %d updated, %d deleted, %d skipped",
               updated_count, deleted_count, skipped_count);
}

void SensorCoveragePlanner3D::DoorCloudCallback(
    const sensor_msgs::PointCloud2::ConstPtr& door_cloud_msg) {
  if (!initialized_)
  {
    return;
  }
  if (door_cloud_msg->data.empty()) {
    return;
  }
  // reset the adjacency_matrix
  adjacency_matrix.setZero();
  door_cloud_->points.clear();
  door_cloud_vis_->cloud_->points.clear();
      std::set<int>
          room_ids;
  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr door_cloud_tmp(
      new pcl::PointCloud<pcl::PointXYZRGBL>());
  pcl::fromROSMsg(*door_cloud_msg, *door_cloud_tmp);

  // only keep the door cloud that are not in collision
  int room_id_0, room_id_1;
  for (auto &point : door_cloud_tmp->points)
  {
    room_ids.insert(point.r);
    room_ids.insert(point.g);
    point.z = robot_position_.z; // set the z coordinate to the robot position z
    if (!planning_env_->DoorInCollision(point.x, point.y, point.z))
    {
      door_cloud_->points.push_back(point);
      pcl::PointXYZRGBL door_point_vis;
      door_point_vis.x = point.x;
      door_point_vis.y = point.y;
      door_point_vis.z = point.z;

      Eigen::Vector3d color_1 = misc_utils_ns::idToColor(point.r);
      Eigen::Vector3d color_2 = misc_utils_ns::idToColor(point.g);
      // find the average color of the two rooms
      door_point_vis.b = (color_1.x() + color_2.x()) / 2 * 255;
      door_point_vis.g = (color_1.y() + color_2.y()) / 2 * 255;
      door_point_vis.r = (color_1.z() + color_2.z()) / 2 * 255;
      door_point_vis.label = point.label; // label
      door_cloud_vis_->cloud_->points.push_back(door_point_vis);
    }
  }

  door_cloud_vis_->Publish();

  for (auto &point : door_cloud_->points)
  {
      room_id_0 = point.r;
      room_id_1 = point.g;
      adjacency_matrix(room_id_0 - 1, room_id_1 - 1) = 1;
      adjacency_matrix(room_id_1 - 1, room_id_0 - 1) = 1;
  }
}

void SensorCoveragePlanner3D::RoomNodeListCallback(
    const tare_planner::RoomNodeList::ConstPtr& room_node_list_msg)
{
  if (!initialized_)
  {
    return;
  }
  if (room_node_list_msg->nodes.empty())
  {
    ROS_ERROR("Room node list is empty");
    return;
  }
  for (auto &id_to_room_node : representation_->GetRoomNodesMapMutable())
  {
    id_to_room_node.second.SetAlive(false);
  }
  for (const auto &room_node_msg : room_node_list_msg->nodes) {
    // 如果 room node 不存在，使用 AddRoomNode 创建新的
    if (!representation_->HasRoomNode(room_node_msg.id)) {
      representation_->AddRoomNode(room_node_msg);
    } else {
      representation_->GetRoomNode(room_node_msg.id).UpdateRoomNode(room_node_msg);
    }
  }
  for (auto it = representation_->GetRoomNodesMapMutable().begin(); it != representation_->GetRoomNodesMapMutable().end();)
  {
    if (!it->second.IsAlive())
    {
      it = representation_->GetRoomNodesMapMutable().erase(it);
      ROS_INFO("Room node %d is not alive, removing it",
                 it->first);
    }
    else
    {
      ++it;
    }
  }

  // check the anchor point of each room node, if the room is split into multiple parts, the anchor point may not be in the room
  for (auto &id_to_room_node : representation_->GetRoomNodesMapMutable())
  {
    int room_id = id_to_room_node.first;
    auto &room_node = id_to_room_node.second;
    std::string room_label = room_node.GetRoomLabel();
    // ROS_INFO("Room ID: %d", room_id);
    if (!room_node.IsLabeled())
    {
      continue;
    }
    Eigen::Vector3f anchor_point(room_node.anchor_point_.x, room_node.anchor_point_.y, room_node.anchor_point_.z);
    Eigen::Vector3i anchor_point_voxel = misc_utils_ns::point_to_voxel(anchor_point, shift_, 1.0 / room_resolution_);
    if (anchor_point_voxel.x() < 0 || anchor_point_voxel.x() >= room_mask_.rows ||
        anchor_point_voxel.y() < 0 || anchor_point_voxel.y() >= room_mask_.cols)
    {
      ROS_ERROR("Anchor point of room %d is out of room mask bounds", room_id);
      room_node.ClearRoomLabels();
      continue;
    }
    int room_id_in_mask = room_mask_.at<int>(anchor_point_voxel.x(), anchor_point_voxel.y());
    if (room_id_in_mask != room_id)
    {
      ROS_ERROR("Anchor point of room %d is not in the room, removing labels", room_id);
      room_node.ClearRoomLabels();
    }
  }
}

void SensorCoveragePlanner3D::RoomMaskCallback(
    const sensor_msgs::Image::ConstPtr& room_mask_msg)
{
  if (!initialized_)
  {
    return;
  }
  if (room_mask_msg->data.empty())
  {
    return;
  }
  if (room_voxel_dimension_.x() <= 0 || room_voxel_dimension_.y() <= 0)
  {
    ROS_ERROR_THROTTLE(5.0, "Invalid room_voxel_dimension (%d, %d), skip room mask update",
                       room_voxel_dimension_.x(), room_voxel_dimension_.y());
    return;
  }
  if (room_mask_msg->width <= 0 || room_mask_msg->height <= 0)
  {
    return;
  }
  // store the current room mask to room_mask_prev
  // convert the room mask to a cv::Mat
  cv::Mat room_mask(room_mask_msg->height, room_mask_msg->width, CV_32S,
                    const_cast<uint8_t *>(room_mask_msg->data.data()));
  // resize the room mask to the room voxel dimension
  if (room_mask.cols != room_voxel_dimension_.x() || room_mask.rows != room_voxel_dimension_.y())
  {
    cv::resize(room_mask, room_mask, cv::Size(room_voxel_dimension_.x(), room_voxel_dimension_.y()),
               0, 0, cv::INTER_NEAREST);
  }
  // update the room mask
  room_mask.copyTo(room_mask_);

  viewpoint_manager_->SetRoomMask(room_mask_);
  grid_world_->SetRoomMask(room_mask_);
  if (initialized_ && representation_)
  {
    representation_->UpdateViewpointRoomIdsFromMask(room_mask_, shift_, room_resolution_);
  }
}

void SensorCoveragePlanner3D::GoalPointCallback(
  const geometry_msgs::PointStamped::ConstPtr& goal_point_msg)
{
  if (!initialized_)
  {
    return;
  }
  // Check if current room id is initialized
  if (current_room_id_ == -1)
  {
    ROS_ERROR("Current room id is -1, not initialized");
    return;
  }

  // Convert goal point to Eigen
  Eigen::Vector3f goal_position_float(
    goal_point_msg->point.x,
    goal_point_msg->point.y,
    goal_point_msg->point.z);
  Eigen::Vector3i goal_position_voxel = misc_utils_ns::point_to_voxel(
    goal_position_float, shift_, 1.0 / room_resolution_);

  // Check bounds
  if (goal_position_voxel.x() < 0 || goal_position_voxel.x() >= room_mask_.rows ||
    goal_position_voxel.y() < 0 || goal_position_voxel.y() >= room_mask_.cols)
  {
    ROS_ERROR("Goal point is out of room mask bounds");
    return;
  }

  geometry_msgs::Point goal_position;
  goal_position.x = goal_point_msg->point.x;
  goal_position.y = goal_point_msg->point.y;
  goal_position.z = goal_point_msg->point.z;

  target_room_id_ = room_mask_.at<int>(goal_position_voxel.x(), goal_position_voxel.y());
  int mask_room_id = target_room_id_;
  if (mask_room_id <= 0 && candidate_room_id_ > 0 &&
      representation_->HasRoomNode(candidate_room_id_))
  {
    ROS_WARN("Goal anchor maps to room mask id %d, using candidate room id %d instead",
             mask_room_id, candidate_room_id_);
    target_room_id_ = candidate_room_id_;
  }
  else if (mask_room_id > 0)
  {
    target_room_id_ = mask_room_id;
  }
  candidate_room_id_ = -1;

  if (target_room_id_ <= 0)
  {
    ROS_ERROR("Invalid target room id %d, cannot start room transit", target_room_id_);
    return;
  }

  // If goal is in the same room, no need to transit
  if (target_room_id_ == current_room_id_ && !transit_across_room_)
  {
    ROS_INFO("Goal point is in the same room as the robot");
    ResetRoomInfo();
    return;
  }

  transit_across_room_ = true;

  // Update goal position
  goal_position_.x = goal_point_msg->point.x;
  goal_position_.y = goal_point_msg->point.y;
  goal_position_.z = goal_point_msg->point.z;

  // republish the goal point using room_anchor_point_pub_
  geometry_msgs::PointStamped goal_point_repub;
  goal_point_repub.header.frame_id = "map";
  goal_point_repub.header.stamp = ros::Time::now();
  goal_point_repub.point = goal_position;
  room_anchor_point_pub_.publish(goal_point_repub);
}

void SensorCoveragePlanner3D::RoomTypeCallback(
    const tare_planner::RoomType::ConstPtr& room_type_msg)
{
  Eigen::Vector3f anchor_point(
      room_type_msg->anchor_point.x, room_type_msg->anchor_point.y,
      room_type_msg->anchor_point.z);
  Eigen::Vector3i anchor_point_voxel = misc_utils_ns::point_to_voxel(
      anchor_point, shift_, 1.0 / room_resolution_);
  if (anchor_point_voxel.x() < 0 || anchor_point_voxel.x() >= room_mask_.rows ||
      anchor_point_voxel.y() < 0 || anchor_point_voxel.y() >= room_mask_.cols)
  {
    ROS_ERROR("Anchor point is out of room mask bounds");
    return;
  }
  int room_id = room_mask_.at<int>(anchor_point_voxel.x(),
                                   anchor_point_voxel.y());
  if (!representation_->HasRoomNode(room_id))
  {
    ROS_ERROR("Room id %d is out of bounds",
                 room_id);
    return;
  }
  std::string room_type = room_type_msg->room_type;
  std::string current_room_type_ = representation_->GetRoomNode(room_id).GetRoomLabel();
  ROS_INFO("Room id: %d, Room type: %s, Current room type: %s",
              room_id, room_type.c_str(), current_room_type_.c_str());
  representation_->GetRoomNode(room_id).GetLabelsMutable()[room_type] += room_type_msg->voxel_num; // accumulate the voxel number for each label
  representation_->GetRoomNode(room_id).SetIsLabeled(true); // mark the room as labeled
  std::string current_room_type_new_ = representation_->GetRoomNode(room_id).GetRoomLabel();
  // mark all objects in this room as not considered
  // if the room_type != current_room_type_, then mark all objects in this room as not considered
  if (current_room_type_new_ != current_room_type_)
  {
    for (auto object_id : representation_->GetRoomNode(room_id).GetObjectIndices())
    {
      if (representation_->HasObjectNode(object_id)) {
        representation_->GetObjectNodeRep(object_id).SetIsConsidered(false);
      }
    }
  }
}


void SensorCoveragePlanner3D::KeyboardInputCallback(const std_msgs::String::ConstPtr& keyboard_input_msg)
{
  if (keyboard_input_msg->data == "next")
  {
    if (found_object_)
    {
      ResetFoundObjectInfo();
      ResetFoundAnchorObjectInfo();

      if (!representation_->HasObjectNode(found_object_id_)) {
        ROS_WARN("Found object with id %d does not exist in representation, skip marking as considered", found_object_id_);
        return;
      }
      auto &object_node = representation_->GetObjectNodeRep(found_object_id_);
      object_node.SetIsConsidered(true);
      object_node.SetIsConsideredStrong(true);
      considered_object_ids_.insert(found_object_id_);
    }
  }
  if (keyboard_input_msg->data == "dynamic")
  {
    dynamic_environment_ = true;
    ROS_INFO("✅✅✅✅✅✅Dynamic environment mode on");
  }
  if (keyboard_input_msg->data == "reset")
  {
    tmp_flag_ = true;
  }
}

void SensorCoveragePlanner3D::TargetObjectInstructionCallback(
    const tare_planner::TargetObjectInstruction::ConstPtr& target_object_instruction_msg)
{
  if (!initialized_)
  {
    return;
  }
  target_object_ = target_object_instruction_msg->target_object;
  room_condition_ = target_object_instruction_msg->room_condition;
  spatial_condition_ = target_object_instruction_msg->spatial_condition;
  anchor_object_ = target_object_instruction_msg->anchor_object;
  attribute_condition_ = target_object_instruction_msg->attribute_condition;
  last_target_object_instruction_time_ = ros::Time::now();

  // reset considered object ids
  for (auto &object_id_pair : representation_->GetObjectNodeRepMapMutable())
  {
    object_id_pair.second.SetIsConsidered(false);
    object_id_pair.second.SetIsConsideredStrong(false);
  }
  ResetFoundObjectInfo();
  ResetFoundAnchorObjectInfo();
  // reset the times of asked for all rooms
  for (auto &id_room_pair : representation_->GetRoomNodesMapMutable())
  {
    id_room_pair.second.SetIsAskedValue(2);
  }
}

void SensorCoveragePlanner3D::TargetObjectCallback(
    const tare_planner::TargetObject::ConstPtr& target_object_msg)
{
  if (!initialized_)
  {
    return;
  }
  int candidate_found_object_id = target_object_msg->object_id;
  if (!representation_->HasObjectNode(candidate_found_object_id))
  {
    ROS_ERROR("Target object id %d is out of bounds",
                 candidate_found_object_id);
    return;
  }
  representation_ns::ObjectNodeRep &candidate_found_object = representation_->GetObjectNodeRep(candidate_found_object_id);
  geometry_msgs::Point candidate_found_object_position_ = candidate_found_object.GetPosition();
  int candidate_found_object_room_id_ = candidate_found_object.room_id_;

  // if (candidate_found_object.IsConsidered() || candidate_found_object.IsConsideredStrong())
  if (candidate_found_object.IsConsideredStrong())
  {
    // already considered this object, skip
    return;
  }
  if (ros::Time(target_object_msg->header.stamp) < last_target_object_instruction_time_)
  {
    // the target object message is from previous instruction, skip
    return;
  }

  nav_msgs::Path path;
  double candidate_found_object_distance = keypose_graph_->GetShortestPath(robot_position_, candidate_found_object_position_, false, path, true);

  if (target_object_msg->is_target)
  {
    if (!found_object_)
    {
      found_object_ = true;
      found_object_id_ = candidate_found_object_id;
      found_object_position_ = candidate_found_object_position_;
      found_object_room_id_ = candidate_found_object_room_id_;
      found_object_distance_ = candidate_found_object_distance;
      ask_found_object_ = false;

      // Override the transit across room state
      ResetRoomInfo();

      ROS_INFO("✅✅✅ Found target object id: %d", found_object_id_);
    }
    else
    {
      if (candidate_found_object_distance < found_object_distance_)
      {
        found_object_id_ = candidate_found_object_id;
        found_object_position_ = candidate_found_object_position_;
        found_object_room_id_ = candidate_found_object_room_id_;
        found_object_distance_ = candidate_found_object_distance;
        ask_found_object_ = false;

        ROS_INFO("✅✅✅ Update to a closer target object id: %d", found_object_id_);
      }
    }
  }
}

void SensorCoveragePlanner3D::AnchorObjectCallback(
    const tare_planner::TargetObject::ConstPtr& anchor_object_msg)
{
  if (!initialized_)
  {
    return;
  }
  int anchor_object_id = anchor_object_msg->object_id;
  if (!representation_->HasObjectNode(anchor_object_id))
  {
    ROS_ERROR("Anchor object id %d is out of bounds",
                 anchor_object_id);
    return;
  }
  representation_ns::ObjectNodeRep &anchor_object = representation_->GetObjectNodeRep(anchor_object_id);
  geometry_msgs::Point anchor_object_position_ = anchor_object.GetPosition();
  int anchor_object_room_id_ = anchor_object.room_id_;

  nav_msgs::Path path;
  double anchor_object_distance = keypose_graph_->GetShortestPath(robot_position_, anchor_object_position_, false, path, true);

  if (anchor_object_msg->is_target)
  {
    found_anchor_object_ = true;
    found_anchor_object_id_ = anchor_object_id;
    found_anchor_object_position_ = anchor_object_position_;
    found_anchor_object_room_id_ = anchor_object_room_id_;
    found_anchor_object_distance_ = anchor_object_distance;

    ROS_INFO("🔖🔖🔖 Found anchor object id: %d", found_anchor_object_id_);
  }
}

// ================== Set and Reset Found Object Info ==================
void SensorCoveragePlanner3D::SetFoundTargetObject()
{
  grid_world_->SetObjectFound(true);
  grid_world_->SetFoundObjectPosition(found_object_position_);
  local_coverage_planner_->SetObjectFound(true);
}

void SensorCoveragePlanner3D::ResetFoundObjectInfo()
{
  found_object_ = false;
  ask_found_object_ = false;
  found_object_id_ = -1;
  found_object_room_id_ = -1;
  found_object_distance_ = -1.0;
  found_object_position_.x = 0.0;
  found_object_position_.y = 0.0;
  found_object_position_.z = 0.0;

  grid_world_->SetObjectFound(false);
  local_coverage_planner_->SetObjectFound(false);
}

void SensorCoveragePlanner3D::SetFoundAnchorObject()
{
  grid_world_->SetAnchorObjectFound(true);
  grid_world_->SetFoundAnchorObjectPosition(found_anchor_object_viewpoint_positions_);
  local_coverage_planner_->SetAnchorObjectFound(true);
}

void SensorCoveragePlanner3D::ResetFoundAnchorObjectInfo()
{
  found_anchor_object_ = false;
  found_anchor_object_id_ = -1;
  found_anchor_object_room_id_ = -1;
  found_anchor_object_distance_ = -1.0;
  found_anchor_object_position_.x = 0.0;
  found_anchor_object_position_.y = 0.0;
  found_anchor_object_position_.z = 0.0;

  found_anchor_object_viewpoint_positions_.clear();
  found_anchor_object_viewpoint_positions_visited_.clear();

  grid_world_->SetAnchorObjectFound(false);
  local_coverage_planner_->SetAnchorObjectFound(false);
}

// ================== Set and Reset Room Info ==================
void SensorCoveragePlanner3D::LogRoomFinishedStatus()
{
  std::string status_str;
  bool first = true;
  for (const auto& pair : representation_->GetRoomNodesMap())
  {
    if (!first)
    {
      status_str += ", ";
    }
    first = false;
    status_str += "room " + std::to_string(pair.first) + " finished: " +
                  (pair.second.IsCovered() ? "1" : "0");
  }
  ROS_INFO("%s", status_str.c_str());
}

void SensorCoveragePlanner3D::SetStartAndEndRoomId()
{
  if (target_room_id_ <= 0 || !representation_->HasRoomNode(target_room_id_))
  {
    ROS_ERROR("Invalid target room id %d for path planning", target_room_id_);
    ResetRoomInfo();
    return;
  }

  if (target_room_id_ == current_room_id_ && !transit_across_room_)
  {
    ROS_INFO("Goal point is in the same room as the robot");
    ResetRoomInfo();
    return;
  }

  ROS_INFO("Current room id: %d, Target room id: %d",
              current_room_id_, target_room_id_);

  // find a feasible path from the current room to the target room using the adjacency matrix
  std::vector<int> path = misc_utils_ns::find_path_bfs(current_room_id_ - 1, target_room_id_ - 1, adjacency_matrix);

  // print the path
  if (path.empty() || path.size() < 2)
  {
    if (door_position_.x() != -10000.0)
    {
      ROS_ERROR("No path found from current room to target room, continue using the last room position");
      return;
    }
    else
    {
      ROS_ERROR("No path found from current room to target room");
      ResetRoomInfo();
      return;
    }
  }
  std::string path_str;
  for (int i = 0; i < static_cast<int>(path.size()); i++)
  {
    if (i > 0)
    {
      path_str += " -> ";
    }
    path_str += std::to_string(path[i] + 1);
  }
  ROS_INFO("Path from current room to target room: %s", path_str.c_str());
  LogRoomFinishedStatus();
  // 取出path上最后两个房间的id作为起始和结束房间id
  start_room_id_ = path[path.size() - 2] + 1; // +1 to convert to 1-based index
  end_room_id_ = path[path.size() - 1] + 1;   // +1 to convert to 1-based index

  SetRoomPosition(start_room_id_, end_room_id_);

  // if (transit_across_room_ && !room_transit_baseline_captured_ && start_room_id_ > 0)
  // {
  //   CaptureRoomTransitExploringCellsBaseline(start_room_id_);
  //   room_transit_baseline_captured_ = true;
  // }

  return;
}

void SensorCoveragePlanner3D::ResetRoomInfo()
{
  transit_across_room_ = false;

  door_position_.x() = -10000.0;
  door_position_.y() = -10000.0;
  door_position_.z() = -10000.0;

  goal_position_.x = -10000.0; // reset goal position
  goal_position_.y = -10000.0;
  goal_position_.z = -10000.0;

  grid_world_->SetTransitAcrossRoom(transit_across_room_);
  grid_world_->SetRoomPosition(door_position_);
  local_coverage_planner_->SetTransitAcrossRoom(transit_across_room_);
  viewpoint_manager_->SetTransitAcrossRoom(transit_across_room_);

  door_cloud_final_->points.clear();

  at_room_ = false;
  near_room_1_ = false;
  near_room_2_ = false;

  target_room_id_ = -1;
  start_room_id_ = -1;
  end_room_id_ = -1;
  door_normal_ = Eigen::Vector3d(0, 0, 0);


  asked_in_advance_ = false;
  room_navigation_query_counter_ = 0;
  candidate_room_position_ = geometry_msgs::Point();
  candidate_room_id_ = -1;
  has_candidate_room_position_ = false;
  // ClearRoomTransitExploringCellsBaseline();
}

#if 0  // Disabled: cancel room transit during cross-room navigation
void SensorCoveragePlanner3D::ClearRoomTransitExploringCellsBaseline()
{
  room_transit_exploring_cells_baseline_.clear();
  room_transit_baseline_room_id_ = -1;
  room_transit_baseline_captured_ = false;
}

void SensorCoveragePlanner3D::CaptureRoomTransitExploringCellsBaseline(int room_id)
{
  std::vector<int> cell_indices;
  grid_world_->GetExploringCellIndicesInRoom(room_id, cell_indices);
  room_transit_exploring_cells_baseline_.clear();
  room_transit_exploring_cells_baseline_.insert(cell_indices.begin(), cell_indices.end());
  room_transit_baseline_room_id_ = room_id;
}

bool SensorCoveragePlanner3D::HasNewExploringCellsInRoom(int room_id) const
{
  if (!room_transit_baseline_captured_ || room_transit_baseline_room_id_ != room_id)
  {
    return false;
  }

  std::vector<int> current_cell_indices;
  grid_world_->GetExploringCellIndicesInRoom(room_id, current_cell_indices);
  for (int cell_index : current_cell_indices)
  {
    if (room_transit_exploring_cells_baseline_.count(cell_index) == 0)
    {
      return true;
    }
  }
  return false;
}

void SensorCoveragePlanner3D::CancelRoomTransit(int departing_room_id)
{
  transit_across_room_ = false;
  grid_world_->SetTransitAcrossRoom(false);
  grid_world_->SetRoomPosition(Eigen::Vector3d(-10000.0, -10000.0, -10000.0));
  local_coverage_planner_->SetTransitAcrossRoom(false);
  viewpoint_manager_->SetTransitAcrossRoom(false);

  door_position_.x() = -10000.0;
  door_position_.y() = -10000.0;
  door_position_.z() = -10000.0;
  door_normal_ = Eigen::Vector3d(0.0, 0.0, 0.0);
  door_cloud_final_->points.clear();

  goal_position_.x = -10000.0;
  goal_position_.y = -10000.0;
  goal_position_.z = -10000.0;
  target_room_id_ = -1;
  start_room_id_ = -1;
  end_room_id_ = -1;

  at_room_ = false;
  near_room_1_ = false;
  near_room_2_ = false;
  has_candidate_room_position_ = false;
  candidate_room_id_ = -1;
  candidate_room_position_ = geometry_msgs::Point();
  stayed_in_room_counter_ = 0;
  ClearRoomTransitExploringCellsBaseline();

  (void)departing_room_id;
}

bool SensorCoveragePlanner3D::MarkDepartingRoomUnfinishedIfNeeded()
{
  if (!transit_across_room_ || at_room_)
  {
    return false;
  }

  int departing_room_id = start_room_id_ > 0 ? start_room_id_ : current_room_id_;
  if (departing_room_id <= 0)
  {
    return false;
  }

  if (!HasNewExploringCellsInRoom(departing_room_id))
  {
    return false;
  }

  ROS_INFO("Found new exploring cell in room %d during transit; canceling room switch",
           departing_room_id);

  if (representation_->HasRoomNode(departing_room_id))
  {
    representation_->GetRoomNode(departing_room_id).SetIsCovered(false);
  }

  suppress_room_transit_after_cancel_ = true;
  suppress_room_transit_room_id_ = departing_room_id;
  CancelRoomTransit(departing_room_id);
  return true;
}
#endif

void SensorCoveragePlanner3D::SetCurrentRoomId()
{
  enter_wrong_room_ = false;
  viewpoint_manager_->SetEnterWrongRoom(enter_wrong_room_);
  // find the current room id
  Eigen::Vector3f robot_position_tmp(robot_position_.x, robot_position_.y, robot_position_.z);
  Eigen::Vector3f robot_position_old_tmp(robot_position_old_.x, robot_position_old_.y, robot_position_old_.z);
  Eigen::Vector3i robot_position_voxel_new = misc_utils_ns::point_to_voxel(
      robot_position_tmp, shift_, 1.0 / room_resolution_);
  Eigen::Vector3i robot_position_voxel_old = misc_utils_ns::point_to_voxel(
      robot_position_old_tmp, shift_, 1.0 / room_resolution_);
  int room_id_tmp_ = room_mask_.at<int>(robot_position_voxel_new.x(),
                                        robot_position_voxel_new.y());
  int last_room_id_tmp_ = room_mask_.at<int>(robot_position_voxel_old.x(),
                                             robot_position_voxel_old.y());

  if (room_id_tmp_ <= 0)
  {
    ROS_INFO("Robot is not in any room");
    return; // maybe just across a door, no need to update the room id
  }

  if (representation_->HasRoomNode(room_id_tmp_))
  {
    representation_->GetRoomNode(room_id_tmp_).SetIsVisited(true);
  }

  if (transit_across_room_ || found_object_)
  {
    // If the robot is transiting across rooms, we can update the room id immediately
    current_room_id_ = room_id_tmp_;
    robot_position_old_ = robot_position_;
    room_mask_old_ = room_mask_.clone();
    viewpoint_manager_->SetCurrentRoomId(current_room_id_);
    grid_world_->SetCurrentRoomId(current_room_id_);
    ROS_INFO("Current room id: %d", current_room_id_);
    return;
  }

  if ((room_id_tmp_ == current_room_id_) || current_room_id_ == -1)
  {
    // If the robot is still in the same room, no need to update the room id
    current_room_id_ = room_id_tmp_;
    robot_position_old_ = robot_position_;
    room_mask_old_ = room_mask_.clone();
    viewpoint_manager_->SetCurrentRoomId(current_room_id_);
    grid_world_->SetCurrentRoomId(current_room_id_);
    return;
  }
  // 意外走进新房间分为两种大情况：
  // 1. 机器人在走入该房间前就已经知道该房间是一个新房间（即虽然room_id_tmp_!= current_room_id_，但在room_mask_old_上该位置的值也与current_room_id_不同），对于这种情况，一定要绕回原房间
  // 2. 机器人在走入该房间前并不知道该房间是一个新房间（即room_id_tmp_ != current_room_id_，但在room_mask_old_上该位置的值与current_room_id_相同），对于这种情况，需要分类讨论
  //    2.1 因为merge导致的房间id变化，这种情况可以立即更新房间id
  //    2.2 因为split导致的房间id变化，这种情况需要设置false为true，等待VLM的确认
  // 新旧的第一个下标表示mask新旧，第二个下标表示robot position新旧
  if (room_id_tmp_ != current_room_id_)
  {
    int room_label_old_new = room_mask_old_.at<int>(robot_position_voxel_new.x(),
                                                    robot_position_voxel_new.y());
    if (room_label_old_new != current_room_id_)
    {
      // 机器人在走入该房间前就已经知道该房间是一个新房间，一定要绕回原房间。
      ROS_INFO("Robot enters a wrong room %d, but already knows it is a new room before entering.", room_id_tmp_);
      enter_wrong_room_ = true;
      viewpoint_manager_->SetEnterWrongRoom(enter_wrong_room_);
      return;
    }
    else
    {
      // 机器人在走入该房间前并不知道该房间是一个新房间
      cv::Mat room_mask_new_new = (room_mask_ == room_id_tmp_);
      cv::Mat room_mask_old_old = (room_mask_old_ == current_room_id_);
      cv::Mat room_mask_and;
      cv::bitwise_and(room_mask_new_new, room_mask_old_old, room_mask_and);
      int num_1 = cv::countNonZero(room_mask_and);
      int num_2 = cv::countNonZero(room_mask_old_old);
      // 1. 被merge（新id在新mask上的mask and 旧id在旧mask上的mask 占据 旧id在旧mask上的mask的绝大部分） 
      // 2. 被分割出去（新id在新mask上的mask and 旧id在旧mask上的mask 占据 旧id在旧mask上的mask的一小部分部分）
      if (num_1 > num_2 * 0.8)
      {
        // 1. 被merge
        // TODO: 暂时的逻辑

        ROS_INFO("Room %d is merged into room %d", current_room_id_, room_id_tmp_);
        current_room_id_ = room_id_tmp_;
        robot_position_old_ = robot_position_;
        room_mask_old_ = room_mask_.clone();
        viewpoint_manager_->SetCurrentRoomId(current_room_id_);
        grid_world_->SetCurrentRoomId(current_room_id_);
        ROS_INFO("Current room id: %d", current_room_id_);
        return;
      }
      else
      {
        // 2. 被分割出去
        ROS_INFO("Room %d is split into room %d", current_room_id_, room_id_tmp_);
        prev_room_id_ = current_room_id_;
        current_room_id_ = room_id_tmp_;
        robot_position_old_ = robot_position_;
        room_mask_old_ = room_mask_.clone();
        viewpoint_manager_->SetCurrentRoomId(current_room_id_);
        grid_world_->SetCurrentRoomId(current_room_id_);
        return;
      }
    }
  }
}

void SensorCoveragePlanner3D::SetRoomPosition(
    const int &start_room_id, const int &end_room_id)
{
  // 在door_cloud_中找到start_room_id和end_room_id之间的门
  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr door_cloud_final_tmp(
      new pcl::PointCloud<pcl::PointXYZRGBL>());
  for (auto &point : door_cloud_->points)
  {
    if (((point.r == start_room_id && point.g == end_room_id) ||
          (point.r == end_room_id && point.g == start_room_id)) &&
        point.label == 0)
    {
      door_cloud_final_tmp->points.push_back(point);
    }
  }
  if (door_cloud_final_tmp->points.empty())
  {
    if (door_cloud_final_->points.empty())
    {
      ROS_ERROR("No door found between current room and target room, door cloud is empty");
      ResetRoomInfo();
      return;
    }
    else
    {
      ROS_WARN("No door found between current room and target room, using the last door cloud");
    }
  }
  else
  {
    // clear the door cloud final and copy the points from door_cloud_final_tmp
    door_cloud_final_->points.clear();
    door_cloud_final_->points = door_cloud_final_tmp->points;
  }

  // use the center of the door cloud as the goal point
  Eigen::Vector3d door_center(0.0, 0.0, 0.0);
  GetDoorCentroid(door_cloud_final_, door_center);
  // set the door_position_ to the door center
  door_position_.x() = door_center.x();
  door_position_.y() = door_center.y();
  door_position_.z() = robot_position_.z; // keep the z coordinate same as the robot position

  geometry_msgs::PointStamped door_position_msg;
  door_position_msg.header.frame_id = "map";
  door_position_msg.header.stamp = ros::Time::now();
  door_position_msg.point.x = door_position_.x();
  door_position_msg.point.y = door_position_.y();
  door_position_msg.point.z = door_position_.z();
  door_position_pub_.publish(door_position_msg);

  GetDoorNormal(start_room_id_, end_room_id_, door_center, door_normal_);
  // publish the room normal vector
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map"; // 或其他坐标系
  marker.header.stamp = ros::Time::now();
  marker.ns = "normals";
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;

  geometry_msgs::Point start_point;
  start_point.x = door_position_.x();
  start_point.y = door_position_.y();
  start_point.z = door_position_.z();

  geometry_msgs::Point end_point;
  end_point.x = door_position_.x() + door_normal_.x() * 2.0;
  end_point.y = door_position_.y() + door_normal_.y() * 2.0;
  end_point.z = door_position_.z() + door_normal_.z() * 2.0;

  marker.points.push_back(start_point);
  marker.points.push_back(end_point);

  marker.scale.x = 0.1; // 更粗的箭杆
  marker.scale.y = 0.2; // 更大的箭头
  marker.scale.z = 0.3; // 更长的箭头
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration(8); // 设置生存时间
  door_normal_pub_.publish(marker);

  // publish the chosen room boundary
  if (!representation_->HasRoomNode(end_room_id)) {
    ROS_WARN("End room with id %d does not exist in representation, cannot publish boundary", end_room_id);
    return;
  }
  auto &room_node = representation_->GetRoomNode(end_room_id);
  visualization_msgs::Marker room_boundary_marker;
  room_boundary_marker.header.frame_id = "map"; // 或其他坐标系
  room_boundary_marker.header.stamp = ros::Time::now();
  room_boundary_marker.ns = "chosen_room_boundary";
  room_boundary_marker.id = 0;
  room_boundary_marker.type = visualization_msgs::Marker::LINE_STRIP;
  room_boundary_marker.action = visualization_msgs::Marker::ADD;
  const auto &poly = room_node.GetPolygon();
  room_boundary_marker.scale.x = 0.5; // 线宽
  room_boundary_marker.color.b = 1.0;
  room_boundary_marker.color.g = 1.0;
  room_boundary_marker.color.r = 1.0;
  room_boundary_marker.color.a = 1.0;

  for (const auto &pt : poly.polygon.points)
  {
    geometry_msgs::Point p;
    p.x = pt.x;
    p.y = pt.y;
    p.z = robot_position_.z-0.1; // 使用机器人的z坐标作为高度
    room_boundary_marker.points.push_back(p);
  }

  // 闭合多边形
  if (!room_boundary_marker.points.empty())
  {
    room_boundary_marker.points.push_back(room_boundary_marker.points.front());
  }
  room_boundary_marker.lifetime = ros::Duration(5); // 设置生存时间
  chosen_room_boundary_pub_.publish(room_boundary_marker);

  grid_world_->SetTransitAcrossRoom(transit_across_room_);
  grid_world_->SetRoomPosition(door_position_);
  local_coverage_planner_->SetTransitAcrossRoom(transit_across_room_);
  viewpoint_manager_->SetTransitAcrossRoom(transit_across_room_);
}

void SensorCoveragePlanner3D::GetDoorCentroid(const int& start_room_id, const int& end_room_id, Eigen::Vector3d& door_center)
{
  door_center.setZero();
  int door_count = 0;
  for (auto &point : door_cloud_->points)
  {
    if ((point.r == start_room_id && point.g == end_room_id) ||
        (point.g == start_room_id && point.r == end_room_id))
    {
      door_center.x() += point.x;
      door_center.y() += point.y;
      door_center.z() += point.z;
      door_count++;
    }
  }
  door_center /= door_count;
  door_center.z() = robot_position_.z; // keep the z coordinate same as the robot position
}

void SensorCoveragePlanner3D::GetDoorCentroid(const pcl::PointCloud<pcl::PointXYZRGBL>::Ptr door_cloud_final, Eigen::Vector3d& door_center)
{
  door_center.setZero();
  int door_count = 0;
  for (auto &point : door_cloud_final->points)
  {
    door_center.x() += point.x;
    door_center.y() += point.y;
    door_center.z() += point.z;
    door_count++;
  }
  door_center /= door_count;
  door_center.z() = robot_position_.z; // keep the z coordinate same as the robot position
}

void SensorCoveragePlanner3D::GetDoorNormal(const int &start_room_id, const int &end_room_id, const Eigen::Vector3d &door_center, Eigen::Vector3d &door_normal)
{
  auto t_0 = std::chrono::high_resolution_clock::now();

  double door_normal_length = 1.5;
  // 从0度到360度开始每隔10度遍历，记录一个范围满足正方向是end_room_id，反方向是start_room_id，取这个范围的中点作为法向
  std::vector<double>
      valid_directions;
  for (double angle = 0.0; angle < 360.0; angle += 1.0)
  {
    Eigen::Vector3d door_normal_tmp(cos(angle * M_PI / 180.0),
                                sin(angle * M_PI / 180.0), 0.0);

    for (double length = 0.3; length < door_normal_length; length += 0.1)
    {
      Eigen::Vector3d door_normal_point = door_center + door_normal_tmp * length;
      Eigen::Vector3d door_normal_point_reverse = door_center - door_normal_tmp * length;
      Eigen::Vector3f door_normal_point_f(
          door_normal_point.x(),
          door_normal_point.y(),
          door_normal_point.z());

      Eigen::Vector3f door_normal_point_reverse_f(
          door_normal_point_reverse.x(),
          door_normal_point_reverse.y(),
          door_normal_point_reverse.z());

      Eigen::Vector3i door_normal_point_voxel = misc_utils_ns::point_to_voxel(
          door_normal_point_f, shift_, 1.0 / room_resolution_);
      Eigen::Vector3i door_normal_point_reverse_voxel = misc_utils_ns::point_to_voxel(
          door_normal_point_reverse_f, shift_, 1.0 / room_resolution_);
      // 加边界检查
      if (door_normal_point_voxel.x() >= 0 && door_normal_point_voxel.x() < room_mask_.cols &&
          door_normal_point_voxel.y() >= 0 && door_normal_point_voxel.y() < room_mask_.rows &&
          door_normal_point_reverse_voxel.x() >= 0 && door_normal_point_reverse_voxel.x() < room_mask_.cols &&
          door_normal_point_reverse_voxel.y() >= 0 && door_normal_point_reverse_voxel.y() < room_mask_.rows)
      {
        int door_normal_label = room_mask_.at<int>(
            door_normal_point_voxel.x(), door_normal_point_voxel.y()); // OpenCV 是 (row, col)
        int door_normal_reverse_label = room_mask_.at<int>(
            door_normal_point_reverse_voxel.x(), door_normal_point_reverse_voxel.y());

        if (door_normal_label == end_room_id &&
            door_normal_reverse_label == start_room_id)
        {
          // 如果法向指向 end_room_id，则使用该法向
          valid_directions.push_back(angle);
          continue;
        }
      }
    }
  }
  if (valid_directions.size() > 0)
  {
    // 做向量平均而非角度平均
    double sum_x = 0.0;
    double sum_y = 0.0;
    for (double angle : valid_directions)
    {
      sum_x += cos(angle * M_PI / 180.0);
      sum_y += sin(angle * M_PI / 180.0);
    }
    double avg_x = sum_x / valid_directions.size();
    double avg_y = sum_y / valid_directions.size();
    Eigen::Vector3d door_normal_tmp(avg_x, avg_y, 0.0);
    door_normal = door_normal_tmp.normalized();
  }
  else
  {
    door_normal = Eigen::Vector3d(0.0, 0.0, 0.0);
    ROS_WARN("No valid door normal found.");
  }

  auto t_1 = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t_1 - t_0);
  // ROS_INFO("Time taken to compute door normal: %lld ms",
  //             duration.count());
}

void SensorCoveragePlanner3D::CheckDoorCloudInRange()
{
  if (!initialized_)
  {
    ROS_ERROR("Planner not initialized, cannot check door cloud in range");
    return;
  }

  // ROS_INFO("Room Nodes size: %d", representation_->GetRoomNodeCount());

  std::vector<int> in_range_rooms;
  // Check whether there are any door clouds in the local planning horizon
  for (const auto &point : door_cloud_->points)
  {
    Eigen::Vector3d point_pos(point.x, point.y, point.z);
    if (viewpoint_manager_->InLocalPlanningHorizonWithoutRoom(point_pos) && (point.r == current_room_id_ || point.g == current_room_id_))
    {
      int target_room_id = point.r != current_room_id_ ? point.r : point.g;
      in_range_rooms.push_back(target_room_id);
    }
  }
  misc_utils_ns::UniquifyIntVector(in_range_rooms);

  // Force the robot to go to the door to see the new room if the door is in range
  door_cloud_in_range_->cloud_->points.clear();
  for (int room_id : in_range_rooms)
  {
    if (!representation_->HasRoomNode(room_id))
    {
      ROS_WARN("Room with id %d does not exist in representation, skip", room_id);
      continue;
    }
    // Skip fully explored rooms; still guide toward doors of partially seen rooms.
    if (representation_->GetRoomNode(room_id).IsCovered())
    {
      continue;
    }
    Eigen::Vector3d door_center(0.0, 0.0, 0.0);
    GetDoorCentroid(current_room_id_, room_id, door_center);
    if (viewpoint_manager_->InLocalPlanningHorizonWithoutRoom(door_center))
    {
      Eigen::Vector3d door_normal(0.0, 0.0, 0.0);
      GetDoorNormal(current_room_id_, room_id, door_center, door_normal);
      if (door_normal.norm() < 0.1)
      {
        ROS_WARN("Door normal is not valid, skipping room %d", room_id);
        continue;
      }

      Eigen::Vector3d candidate_viewpoint_rep_pos = door_center - door_normal * 0.5;
      int room_candidate_viewpoint_ind = viewpoint_manager_->GetViewPointInd(candidate_viewpoint_rep_pos);
      if (viewpoint_manager_->InRange(room_candidate_viewpoint_ind) && viewpoint_manager_->IsViewPointCandidate(room_candidate_viewpoint_ind))
      {
        pcl::PointXYZLNormal door_pt;
        door_pt.x = candidate_viewpoint_rep_pos.x();
        door_pt.y = candidate_viewpoint_rep_pos.y();
        door_pt.z = candidate_viewpoint_rep_pos.z();
        door_pt.label = room_id;
        door_pt.normal_x = door_normal.x();
        door_pt.normal_y = door_normal.y();
        door_pt.normal_z = door_normal.z();
        door_cloud_in_range_->cloud_->points.push_back(door_pt);
        MY_ASSERT(room_candidate_viewpoint_ind >= 0);
        viewpoint_manager_->SetViewPointRoomCandidate(room_candidate_viewpoint_ind, true);
      }
    }
  }
  if (door_cloud_in_range_->cloud_->points.size() > 0)
  {
    door_cloud_in_range_->Publish();
  }
}

void SensorCoveragePlanner3D::SendInitialWaypoint()
{
  // send waypoint ahead
  double lx = 12.0;
  double ly = 0.0;
  double dx = cos(robot_yaw_) * lx - sin(robot_yaw_) * ly;
  double dy = sin(robot_yaw_) * lx + cos(robot_yaw_) * ly;

  geometry_msgs::PointStamped waypoint;
  waypoint.header.frame_id = "map";
  waypoint.header.stamp = ros::Time::now();
  waypoint.point.x = robot_position_.x + dx;
  waypoint.point.y = robot_position_.y + dy;
  waypoint.point.z = robot_position_.z;
  waypoint_pub_.publish(waypoint);
}

void SensorCoveragePlanner3D::SendInRoomWaypoint()
{
  double distance = 2.5 + room_guide_counter_ * 1.0; // distance to the waypoint
  if (door_normal_.norm() < 0.01) {
    ROS_ERROR("Room normal vector is zero, cannot send projected waypoint. Instead, send the door center as waypoint.");
    geometry_msgs::PointStamped waypoint;
    waypoint.header.frame_id = "map";
    waypoint.header.stamp = ros::Time::now();
    waypoint.point.x = door_position_.x();
    waypoint.point.y = door_position_.y();
    waypoint.point.z = robot_position_.z;
    waypoint_pub_.publish(waypoint);
    return;
  }
  Eigen::Vector3d room_normal = door_normal_.normalized();
  // Calculate the waypoint position based on the room normal vector
  double dx = room_normal.x() * distance;
  double dy = room_normal.y() * distance;

  geometry_msgs::PointStamped waypoint;
  waypoint.header.frame_id = "map";
  waypoint.header.stamp = ros::Time::now();
  waypoint.point.x = door_position_.x() + dx;
  waypoint.point.y = door_position_.y() + dy;
  waypoint.point.z = robot_position_.z;
  ROS_INFO("Send waypoint in room: (%.2f, %.2f, %.2f)",
              waypoint.point.x, waypoint.point.y, waypoint.point.z);
  waypoint_pub_.publish(waypoint);
}

void SensorCoveragePlanner3D::UpdateKeyposeGraph() {
  misc_utils_ns::Timer update_keypose_graph_timer("update keypose graph");
  update_keypose_graph_timer.Start();

  keypose_graph_->GetMarker(keypose_graph_node_marker_->marker_,
                            keypose_graph_edge_marker_->marker_);
  // keypose_graph_node_marker_->Publish();
  keypose_graph_edge_marker_->Publish();
  keypose_graph_vis_cloud_->cloud_->clear();
  keypose_graph_->CheckLocalCollision(robot_position_, viewpoint_manager_);
  keypose_graph_->CheckConnectivity(robot_position_);
  keypose_graph_->GetVisualizationCloud(keypose_graph_vis_cloud_->cloud_);
  keypose_graph_vis_cloud_->Publish();

  update_keypose_graph_timer.Stop(false);
}

int SensorCoveragePlanner3D::UpdateViewPoints() {
  misc_utils_ns::Timer collision_cloud_timer("update collision cloud");
  collision_cloud_timer.Start();
  collision_cloud_->cloud_ = planning_env_->GetCollisionCloud();
  collision_cloud_timer.Stop(false);

  misc_utils_ns::Timer viewpoint_manager_update_timer(
      "update viewpoint manager");
  viewpoint_manager_update_timer.Start();
  if (kUseTerrainHeight) {
    viewpoint_manager_->SetViewPointHeightWithTerrain(
        large_terrain_cloud_->cloud_);
  }
  if (kCheckTerrainCollision) {
    *(collision_cloud_->cloud_) += *(terrain_collision_cloud_->cloud_);
    *(collision_cloud_->cloud_) += *(terrain_ext_collision_cloud_->cloud_);
  }
  viewpoint_manager_->CheckViewPointCollision(collision_cloud_->cloud_);
  viewpoint_manager_->CheckViewPointRoomBoundaryCollision();
  viewpoint_manager_->CheckViewPointLineOfSight();
  viewpoint_manager_->CheckViewPointConnectivity();
  int viewpoint_candidate_count = viewpoint_manager_->GetViewPointCandidate();

  UpdateVisitedPositions();
  viewpoint_manager_->UpdateViewPointVisited(visited_positions_);
  viewpoint_manager_->UpdateViewPointVisited(grid_world_); // only used for multi-robot exploration

  // For visualization
  collision_cloud_->Publish();
  // collision_grid_cloud_->Publish();
  viewpoint_manager_->GetCollisionViewPointVisCloud(
      viewpoint_in_collision_cloud_->cloud_);
  viewpoint_in_collision_cloud_->Publish();

  viewpoint_manager_update_timer.Stop(false);
  return viewpoint_candidate_count;
}

void SensorCoveragePlanner3D::UpdateViewPointCoverage() {
  // Update viewpoint coverage
  misc_utils_ns::Timer update_coverage_timer("update viewpoint coverage");
  update_coverage_timer.Start();
  viewpoint_manager_->UpdateViewPointCoverage<PlannerCloudPointType>(
      planning_env_->GetDiffCloud());
  viewpoint_manager_->UpdateRolledOverViewPointCoverage<PlannerCloudPointType>(
      planning_env_->GetStackedCloud());
  // Update robot coverage
  robot_viewpoint_.ResetCoverage();
  geometry_msgs::Pose robot_pose;
  robot_pose.position = robot_position_;
  robot_viewpoint_.setPose(robot_pose);
  UpdateRobotViewPointCoverage();
  update_coverage_timer.Stop(false);
}

void SensorCoveragePlanner3D::UpdateRobotViewPointCoverage() {
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud =
      planning_env_->GetCollisionCloud();
  for (const auto &point : cloud->points) {
    if (viewpoint_manager_->InFOVAndRange(
            Eigen::Vector3d(point.x, point.y, point.z),
            Eigen::Vector3d(robot_position_.x, robot_position_.y,
                            robot_position_.z))) {
      robot_viewpoint_.UpdateCoverage<pcl::PointXYZI>(point);
    }
  }
}

void SensorCoveragePlanner3D::UpdateCoveredAreas(
    int &uncovered_point_num, int &uncovered_frontier_point_num) {
  // Update covered area
  misc_utils_ns::Timer update_coverage_area_timer("update covered area");
  update_coverage_area_timer.Start();
  planning_env_->UpdateCoveredArea(robot_viewpoint_, viewpoint_manager_);

  update_coverage_area_timer.Stop(false);
  misc_utils_ns::Timer get_uncovered_area_timer("get uncovered area");
  get_uncovered_area_timer.Start();
  planning_env_->GetUncoveredArea(viewpoint_manager_, uncovered_point_num,
                                  uncovered_frontier_point_num);

  get_uncovered_area_timer.Stop(false);
  planning_env_->PublishUncoveredCloud();
  planning_env_->PublishUncoveredFrontierCloud();
}

void SensorCoveragePlanner3D::UpdateVisitedPositions() {
  Eigen::Vector3d robot_current_position(robot_position_.x, robot_position_.y,
                                         robot_position_.z);
  bool existing = false;
  for (int i = 0; i < visited_positions_.size(); i++) {
    // TODO: parameterize this
    if ((robot_current_position - visited_positions_[i]).norm() < 1) {
      existing = true;
      break;
    }
  }
  if (!existing) {
    visited_positions_.push_back(robot_current_position);
  }
}

void SensorCoveragePlanner3D::UpdateObjectVisibility()
{
  std::vector<int> visible_object_ids = {};
  for (auto &object_id : representation_->GetLatestObjectNodeIndicesMutable())
  {
    if (!representation_->HasObjectNode(object_id)) {
      ROS_WARN("Object with id %d does not exist in representation, skip", object_id);
      continue;
    }
    auto &object = representation_->GetObjectNodeRep(object_id);
    geometry_msgs::Point obj_position = object.GetPosition();
    Eigen::Vector3d object_pos(obj_position.x,
                                obj_position.y,
                                obj_position.z);

    auto cloud_msg = object.GetCloud();
    pcl::PointCloud<pcl::PointXYZ>::Ptr obj_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(cloud_msg, *obj_cloud);
    auto voxels = Convert2Voxels(obj_cloud);
    for (auto &viewpoint : representation_->GetViewPointRepsMutable())
    {
      if (viewpoint.HasObjectIndex(object_id))
      {
        object.AddVisibleViewpoint(viewpoint.GetId());
        continue;
      }
      // if timestamp is very close, connect
      // if the viewpoint id is in the object visible viewpoint list, connect
      // if ((std::abs((viewpoint.GetTimestamp() - object.GetTimestamp()).toSec()) < 0.5) || std::find(object.GetVisibleViewpointIndices().begin(),
      //                                                                                         object.GetVisibleViewpointIndices().end(),
      //                                                                                         viewpoint.GetId()) != object.GetVisibleViewpointIndices().end())
      if (std::find(object.GetVisibleViewpointIndices().begin(), object.GetVisibleViewpointIndices().end(), viewpoint.GetId()) != object.GetVisibleViewpointIndices().end())
      {
        viewpoint.AddObjectIndex(object_id);
        viewpoint.AddDirectObjectIndex(object_id);
        object.AddVisibleViewpoint(viewpoint.GetId());
        visible_object_ids.push_back(object_id);

        object.SetIsConsidered(false);
        // ROS_INFO(//             "Object ID %d (%s) is detected at viewpoint %d (new object)",
        //             object.GetObjectId(),
        //             object.GetLabel().c_str(),
        //             viewpoint.GetId());
        continue;
      }
      else
      {
        // do ray-casting to check visibility
        if (viewpoint.HasObjectIndex(object_id))
        {
          visible_object_ids.push_back(object_id);
          continue;
        }
        Eigen::Vector3d current_viewpoint_pos(viewpoint.GetPosition().x,
                                              viewpoint.GetPosition().y,
                                              viewpoint.GetPosition().z + 0.265); // make this to the height of the robot camera
        Eigen::Vector3i curr_viewpoint_voxel = planning_env_->Pos2Sub(current_viewpoint_pos);

        if ((current_viewpoint_pos - object_pos).norm() > rep_sensor_range)
        {
          continue;
        }
        bool is_visible = false; // Initialize visibility flag
        for (auto &pt : voxels)
        {
          is_visible = CheckRayVisibilityInOccupancyGrid(curr_viewpoint_voxel, pt);
          if (is_visible)
          {
            break; // if any voxel is visible, we consider the object visible
          }
        }
        if (is_visible)
        {
          viewpoint.AddObjectIndex(object_id);
          object.AddVisibleViewpoint(viewpoint.GetId());
          visible_object_ids.push_back(object_id);

          object.SetIsConsidered(false);
          // ROS_INFO(//             "Object ID %d (%s) is visible from viewpoint %d",
          //             object.GetObjectId(),
          //             object.GetLabel().c_str(),
          //             viewpoint.GetId());
        }
        // else
        // {
        //   ROS_INFO(//               "Object ID %d (%s) is NOT visible from viewpoint %d",
        //               object.GetObjectId(),
        //               object.GetLabel().c_str(),
        //               viewpoint.GetId());
        // }
      }
    }

    // ---------- set the obj-room relation-----------
    Eigen::Vector3f object_pos_f(obj_position.x,
                                 obj_position.y,
                                 obj_position.z);
    Eigen::Vector3i object_pos_voxel = misc_utils_ns::point_to_voxel(
        object_pos_f, shift_, 1.0 / room_resolution_);
    if (object_pos_voxel.x() >= 0 && object_pos_voxel.x() < room_mask_.cols &&
        object_pos_voxel.y() >= 0 && object_pos_voxel.y() < room_mask_.rows)
    {
      int object_room_id = room_mask_.at<int>(object_pos_voxel.x(),
                                              object_pos_voxel.y());
      if (object_room_id == 0)
      {
        // // if the object is visible from any viewpoint, use the room id of that viewpoint
        // if (!object.GetVisibleViewpointIndices().empty())
        // {
        //   int vp_id = *object.GetVisibleViewpointIndices().begin();
        //   object_room_id = representation_->GetViewPointRepNode(vp_id).GetRoomId();
        // }
        // else
        {
          // if the object is not visible from any viewpoint, we need to dilate the object position by 2 voxels to get a non-zero room id
          int dilation_size = 2;
          bool found = false;
          for (int dx = -dilation_size; dx <= dilation_size && !found; dx++)
          {
            for (int dy = -dilation_size; dy <= dilation_size; dy++)
            {
              int nx = object_pos_voxel.x() + dx;
              int ny = object_pos_voxel.y() + dy;
              if (nx >= 0 && nx < room_mask_.cols &&
                  ny >= 0 && ny < room_mask_.rows)
              {
                int neighbor_room_id = room_mask_.at<int>(nx, ny);
                if (neighbor_room_id > 0)
                {
                  object_room_id = neighbor_room_id;
                  found = true;
                  break;  
                }
              }
            }
          }
        }
      }
      representation_->SetObjectRoomRelation(object_id, object_room_id);
    }
    // ---------- set the obj-room relation-----------
  }

  // check the current position visibility
  obj_score_ = 0.0;
  Eigen::Vector3d current_pos(robot_position_.x, robot_position_.y, robot_position_.z + 0.265);
  Eigen::Vector3i current_voxel = planning_env_->Pos2Sub(current_pos);
  // print the latest_object_node_indices size
  // ROS_ERROR(//                   "Latest object node rep map size: %zu",
  //                   representation_->GetLatestObjectNodeIndices().size());
  for (auto &object_id : representation_->GetLatestObjectNodeIndicesMutable())
  {
    if (!representation_->HasObjectNode(object_id)) {
      ROS_WARN("Object with id %d does not exist in representation, skip", object_id);
      continue;
    }
    auto &object = representation_->GetObjectNodeRep(object_id);
    geometry_msgs::Point obj_position = object.GetPosition();
    Eigen::Vector3d object_pos(obj_position.x,
                                obj_position.y,
                                obj_position.z);
    auto cloud_msg = object.GetCloud();
    pcl::PointCloud<pcl::PointXYZ>::Ptr obj_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(cloud_msg, *obj_cloud);
    auto voxels = Convert2Voxels(obj_cloud);

    if ((current_pos - object_pos).norm() > rep_sensor_range)
    {
      continue;
    }
    bool is_visible = false; // Initialize visibility flag
    for (auto &pt : voxels)
    {
      is_visible = CheckRayVisibilityInOccupancyGrid(current_voxel, pt);
      if (is_visible)
      {
        break; // if any voxel is visible, we consider the object visible
      }
    }
    if (is_visible && object.GetVisibleViewpointIndices().empty())
    {
      obj_score_ += 1.0;
    }
    else if (is_visible && !object.GetVisibleViewpointIndices().empty())
    {
      obj_score_ += 0.2;
    }
  }
      
  misc_utils_ns::UniquifyIntVector(visible_object_ids);
  // remove the visible objects from the latest_object_node_indices
  for (int &object_id : visible_object_ids)
  {
    representation_->GetLatestObjectNodeIndicesMutable().erase(object_id);
  }
}  

void SensorCoveragePlanner3D::UpdateViewpointObjectVisibility()
{
  representation_ns::ViewPointRep &current_viewpoint = representation_->GetViewPointRepNode(curr_viewpoint_rep_node_ind);
  Eigen::Vector3d current_viewpoint_pos(current_viewpoint.GetPosition().x,
                                        current_viewpoint.GetPosition().y,
                                        current_viewpoint.GetPosition().z + 0.265); // make this to the height of the robot camera
  Eigen::Vector3i curr_viewpoint_voxel = planning_env_->Pos2Sub(current_viewpoint_pos);
  for (auto &id_object_pair : representation_->GetObjectNodeRepMapMutable())
  {
    const int &object_id = id_object_pair.first;
    auto &object = id_object_pair.second;
    // if the object is already in the viewpoint's visibility list, skip it
    if (current_viewpoint.HasObjectIndex(object_id))
    {
      continue;
    }
    geometry_msgs::Point obj_position = object.GetPosition();
    Eigen::Vector3d object_pos(obj_position.x,
                                obj_position.y,
                                obj_position.z);

    if ((current_viewpoint_pos - object_pos).norm() > rep_sensor_range)
    {
      continue;
    }
    auto cloud_msg = object.GetCloud();
    pcl::PointCloud<pcl::PointXYZ>::Ptr obj_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(cloud_msg, *obj_cloud);
    auto voxels = Convert2Voxels(obj_cloud);
    bool is_visible = false; // Initialize visibility flag
    for (auto &pt : voxels)
    {
      is_visible = CheckRayVisibilityInOccupancyGrid(curr_viewpoint_voxel, pt);
      if (is_visible)
      {
        break; // if any voxel is visible, we consider the object visible
      }
    }
    if (is_visible)
    {
      current_viewpoint.AddObjectIndex(object.GetObjectId());
      object.AddVisibleViewpoint(current_viewpoint.GetId());

      object.SetIsConsidered(false);
      // ROS_INFO(//             "Object ID %d (%s) is visible from viewpoint %d",
      //             object.GetObjectId(),
      //             object.GetLabel().c_str(),
      //             current_viewpoint.GetId());
    }
    // else
    // {
    //   ROS_INFO(//               "Object ID %d (%s) is NOT visible from viewpoint %d",
    //               object.GetObjectId(),
    //               object.GetLabel().c_str(),
    //               current_viewpoint.GetId());
    // }
  }  
}

// Helper function to check visibility using occupancy grid
bool SensorCoveragePlanner3D::CheckRayVisibilityInOccupancyGrid(const Eigen::Vector3i& start_pos, 
                                                                const Eigen::Vector3i& end_pos) {

  return planning_env_->CheckLineOfSightInOccupancyGrid(start_pos, end_pos);
}

bool SensorCoveragePlanner3D::InRange(const Eigen::Vector3i& voxel_index) const {
  return planning_env_->InRange(voxel_index);
}

std::vector<Eigen::Vector3i> SensorCoveragePlanner3D::Convert2Voxels(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    std::vector<Eigen::Vector3i> voxel_vector;
    for (const auto& point : cloud->points) {
        Eigen::Vector3d pos(point.x, point.y, point.z);
        Eigen::Vector3i voxel_index = planning_env_->Pos2Sub(pos);
        bool is_valid = InRange(voxel_index);
        if (is_valid) {
            voxel_vector.push_back(voxel_index);
        } else {
            ROS_WARN("Voxel index out of range: (%d, %d, %d)", 
                        voxel_index.x(), voxel_index.y(), voxel_index.z());
        }
    }
    // Remove duplicates
    std::sort(voxel_vector.begin(), voxel_vector.end(), [](const Eigen::Vector3i& a, const Eigen::Vector3i& b) {
        if (a.x() != b.x()) return a.x() < b.x();
        if (a.y() != b.y()) return a.y() < b.y();
        return a.z() < b.z();
    });
    voxel_vector.erase(std::unique(voxel_vector.begin(), voxel_vector.end()), voxel_vector.end());
    
    return voxel_vector;
}

void SensorCoveragePlanner3D::CreateVisibilityMarkers() {
    if (!initialized_) {
        return;
    }

    visualization_msgs::MarkerArray marker_array;
    
    visualization_msgs::Marker delete_marker;
    delete_marker.header.frame_id = kWorldFrameID;
    delete_marker.header.stamp = ros::Time::now();
    delete_marker.ns = "visibility_lines";
    delete_marker.id = 0;
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);

    int unique_marker_id = 1;
    
    for (const auto& viewpoint : representation_->GetViewPointReps()) {
        Eigen::Vector3d viewpoint_pos(
            viewpoint.GetPosition().x,
            viewpoint.GetPosition().y,
            viewpoint.GetPosition().z
        );

        const auto& visible_object_indices = viewpoint.GetObjectIndices();
        for (int object_index : visible_object_indices) {
            bool has_object = representation_->HasObjectNode(object_index);
            if (!has_object) {
                ROS_WARN("Object ID %d not found in object_node_rep_map", object_index);
                continue;
            }

            const auto& object_node = representation_->GetObjectNodeRep(object_index);
            geometry_msgs::Point obj_position = object_node.GetPosition();
            Eigen::Vector3d object_pos(obj_position.x, obj_position.y, obj_position.z);

            visualization_msgs::Marker line_marker;
            line_marker.header.frame_id = kWorldFrameID;
            line_marker.header.stamp = ros::Time::now();
            line_marker.ns = "visibility_lines";
            
            line_marker.id = ++unique_marker_id; // Unique ID for each line marker

            line_marker.type = visualization_msgs::Marker::LINE_LIST;
            line_marker.action = visualization_msgs::Marker::ADD;
            
            // Set the duration of the line marker
            // line_marker.lifetime = ros::Duration(10.0);

            geometry_msgs::Point start_point, end_point;
            start_point.x = viewpoint_pos.x();
            start_point.y = viewpoint_pos.y();
            start_point.z = viewpoint_pos.z();
            end_point.x = object_pos.x();
            end_point.y = object_pos.y();
            end_point.z = object_pos.z();

            line_marker.points.push_back(start_point);
            line_marker.points.push_back(end_point);

            line_marker.scale.x = 0.08; // Thinner lines since there will be many
            if (viewpoint.HasDirectObjectIndex(object_index))
            {
                line_marker.color.r = 0.0;
                line_marker.color.g = 0.0;
                line_marker.color.b = 1.0; // Blue for direct visibility
            }
            else
            {
              line_marker.color.r = 0.0;
              line_marker.color.g = 1.0;
              line_marker.color.b = 0.0;
            }
            line_marker.color.a = 0.6; // Slightly transparent

            marker_array.markers.push_back(line_marker);

            ROS_DEBUG("Created visibility line from viewpoint to object %d (%s)",
                object_node.GetObjectId(),
                object_node.GetLabel().c_str()
            );
        }
    }

    // Publish the marker array
    if (!marker_array.markers.empty()) {
        object_visibility_marker_pub_.publish(marker_array);
        // ROS_INFO(//             "Published %zu visibility markers from all viewpoints", 
        //             marker_array.markers.size() - 1); // -1 for DELETEALL marker
    } else {
        ROS_DEBUG("No visibility connections to publish");
    }
}

void SensorCoveragePlanner3D::PublishViewpointRoomIdMarkers() {
    if (!initialized_) {
        return;
    }

    visualization_msgs::MarkerArray marker_array;
    
    // Delete all previous markers
    visualization_msgs::Marker delete_marker;
    delete_marker.header.frame_id = kWorldFrameID;
    delete_marker.header.stamp = ros::Time::now();
    delete_marker.ns = "viewpoint_room_ids";
    delete_marker.id = 0;
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);

    // int marker_id = 1;
    
    for (const auto& viewpoint : representation_->GetViewPointReps()) {
        // Create text marker for room ID
        visualization_msgs::Marker text_marker;
        text_marker.header.frame_id = kWorldFrameID;
        text_marker.header.stamp = ros::Time::now();
        text_marker.ns = "viewpoint_room_ids";
        text_marker.id = viewpoint.GetId() + 1; // Use viewpoint ID as marker ID
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::Marker::ADD;
        
        // Position the text slightly above the viewpoint
        text_marker.pose.position.x = viewpoint.GetPosition().x;
        text_marker.pose.position.y = viewpoint.GetPosition().y;
        text_marker.pose.position.z = viewpoint.GetPosition().z + 0.5; // 0.5m above viewpoint
        
        text_marker.pose.orientation.x = 0.0;
        text_marker.pose.orientation.y = 0.0;
        text_marker.pose.orientation.z = 0.0;
        text_marker.pose.orientation.w = 1.0;
        
        // Set text content
        text_marker.text = "V" + std::to_string(viewpoint.GetId()) + "R" + std::to_string(viewpoint.GetRoomId());
        
        // Set size and color
        text_marker.scale.z = 0.4; // Text height
        // Set color to red
        text_marker.color.r = 1.0;
        text_marker.color.g = 0.0;
        text_marker.color.b = 0.0; // Red text
        text_marker.color.a = 1.0;
        
        // Set lifetime
        // text_marker.lifetime = ros::Duration(10.0);
        
        marker_array.markers.push_back(text_marker);
  
    }

    // Publish the marker array
    if (!marker_array.markers.empty()) {
        viewpoint_room_id_marker_pub_.publish(marker_array);
        ROS_DEBUG("Published %zu viewpoint room ID markers", 
                     (marker_array.markers.size() - 1) / 2); // -1 for DELETEALL, /2 for text+sphere pairs
    }
}

// void SensorCoveragePlanner3D::PublishViewpointObjectVisibility() {
//     std_msgs::String visibility_msg;
//     std::string json_data = "[";  // array of all viewpoints

//     std::unordered_map<int, std::vector<const representation_ns::ObjectNodeRep*>> vp_to_objects;

//     for (const auto& obj : object_node_reps_) {
//         for (const auto& vp : obj.GetVisibleViewpoints()) {
//             vp_to_objects[vp].push_back(&obj);
//         }
//     }
//     bool first_viewpoint = true;
//     for (const auto& [vp_id, obj_list] : vp_to_objects) {
//         if (!first_viewpoint) json_data += ",";
//         json_data += "{";
//         json_data += "\"viewpoint_id\":" + std::to_string(vp_id) + ",";
//         json_data += "\"visible_objects\":[";
        
//         bool first_obj = true;
//         for (const auto* obj_ptr : obj_list) {
//             if (!first_obj) json_data += ",";
//             json_data += "{";
//             json_data += "\"object_id\":" + std::to_string(obj_ptr->GetObjectId()) + ",";
//             json_data += "\"label\":\"" + obj_ptr->GetLabel() + "\",";
//             json_data += "\"total_viewpoints\":" + std::to_string(obj_ptr->GetVisibleViewpoints().size());

//             auto cloud_msg = obj_ptr->GetCloud();
//             pcl::PointCloud<pcl::PointXYZ>::Ptr obj_cloud(new pcl::PointCloud<pcl::PointXYZ>());
//             pcl::fromROSMsg(cloud_msg, *obj_cloud);
//             json_data += ",\"point_count\":" + std::to_string(obj_cloud->points.size());

//             json_data += "}";
//             first_obj = false;
//         }

//         json_data += "]}";
//         first_viewpoint = false;
//     }

//     json_data += "]";

//     visibility_msg.data = json_data;
//     viewpoint_visibility_pub_.publish(visibility_msg);
    
//     ROS_INFO("Published all viewpoint-object visibility");
// }


void SensorCoveragePlanner3D::UpdateGlobalRepresentation() {
  local_coverage_planner_->SetRobotPosition(
      Eigen::Vector3d(robot_position_.x, robot_position_.y, robot_position_.z));
  bool viewpoint_rollover = viewpoint_manager_->UpdateRobotPosition(
      Eigen::Vector3d(robot_position_.x, robot_position_.y, robot_position_.z));
  if (!grid_world_->Initialized() || viewpoint_rollover) {
    grid_world_->UpdateNeighborCells(robot_position_);
  }

  planning_env_->UpdateRobotPosition(robot_position_);
  planning_env_->GetVisualizationPointCloud(point_cloud_manager_neighbor_cloud_->cloud_);
  point_cloud_manager_neighbor_cloud_->Publish();

  // DEBUG
  Eigen::Vector3d pointcloud_manager_neighbor_cells_origin =
      planning_env_->GetPointCloudManagerNeighborCellsOrigin();
  geometry_msgs::PointStamped
      pointcloud_manager_neighbor_cells_origin_point;
  pointcloud_manager_neighbor_cells_origin_point.header.frame_id = "map";
  pointcloud_manager_neighbor_cells_origin_point.header.stamp = ros::Time::now();
  pointcloud_manager_neighbor_cells_origin_point.point.x =
      pointcloud_manager_neighbor_cells_origin.x();
  pointcloud_manager_neighbor_cells_origin_point.point.y =
      pointcloud_manager_neighbor_cells_origin.y();
  pointcloud_manager_neighbor_cells_origin_point.point.z =
      pointcloud_manager_neighbor_cells_origin.z();
  pointcloud_manager_neighbor_cells_origin_pub_.publish(
      pointcloud_manager_neighbor_cells_origin_point);

  if (exploration_finished_ && kNoExplorationReturnHome) {
    planning_env_->SetUseFrontier(false);
  }
  planning_env_->UpdateKeyposeCloud<PlannerCloudPointType>(
      keypose_cloud_->cloud_);

  int closest_node_ind = keypose_graph_->GetClosestNodeInd(robot_position_);
  geometry_msgs::Point closest_node_position =
      keypose_graph_->GetClosestNodePosition(robot_position_);
  grid_world_->SetCurKeyposeGraphNodeInd(closest_node_ind);
  grid_world_->SetCurKeyposeGraphNodePosition(closest_node_position);

  grid_world_->UpdateRobotPosition(robot_position_);
  if (!grid_world_->HomeSet()) {
    grid_world_->SetHomePosition(initial_position_);
  }

  // // Representation
  // // add keypose_cloud_ to point_cloud_all_
  // *(point_cloud_new_->cloud_) = *(keypose_cloud_->cloud_);
  // int size;
  // planning_env_->GetObsVoxelNumber(size);
  // RCLCPP_INFO(
  //     this->get_logger(), "Number of occupied voxels in the occupancy grid: %d",
  //     size);
  // if (size - voxel_num_ > rep_threshold_)
  // {
  //   voxel_num_ = size;
  //   // add the robot position to viewpoint_rep_vis_cloud_
  //   pcl::PointXYZI robot_point;
  //   robot_point.x = robot_position_.x;
  //   robot_point.y = robot_position_.y;
  //   robot_point.z = robot_position_.z;
  //   robot_point.intensity = 1.0; // Set intensity to 1.0 for visibility
  //   viewpoint_rep_vis_cloud_->cloud_->points.push_back(robot_point);
  //   viewpoint_rep_vis_cloud_->Publish();
  // }
  // current_obs_voxel_inds_.clear();
  // planning_env_->GetUpdatedVoxelInds(current_obs_voxel_inds_);
}

void SensorCoveragePlanner3D::GlobalPlanning(
    std::vector<int> &global_cell_tsp_order,
    exploration_path_ns::ExplorationPath &global_path) {
  misc_utils_ns::Timer global_tsp_timer("Global planning");
  global_tsp_timer.Start();

  grid_world_->UpdateCellStatus(viewpoint_manager_);
  grid_world_->UpdateCellKeyposeGraphNodes(keypose_graph_);
  grid_world_->AddPathsInBetweenCells(viewpoint_manager_, keypose_graph_);

  viewpoint_manager_->UpdateCandidateViewPointCellStatus(grid_world_);

  global_path = grid_world_->SolveGlobalTSP(
      viewpoint_manager_, global_cell_tsp_order, keypose_graph_);

  global_tsp_timer.Stop(false);
  global_planning_runtime_ = global_tsp_timer.GetDuration("ms");
}

void SensorCoveragePlanner3D::PublishGlobalPlanningVisualization(
    const exploration_path_ns::ExplorationPath &global_path,
    const exploration_path_ns::ExplorationPath &local_path) {
  nav_msgs::Path global_path_full = global_path.GetPath();
  global_path_full.header.frame_id = "map";
  global_path_full.header.stamp = ros::Time::now();
  global_path_full_publisher_.publish(global_path_full);
  // Get the part that connects with the local path

  int start_index = 0;
  for (int i = 0; i < global_path.nodes_.size(); i++) {
    if (global_path.nodes_[i].type_ ==
            exploration_path_ns::NodeType::GLOBAL_VIEWPOINT ||
        global_path.nodes_[i].type_ == exploration_path_ns::NodeType::HOME ||
        !viewpoint_manager_->InLocalPlanningHorizon(
            global_path.nodes_[i].position_)) {
      break;
    }
    start_index = i;
  }

  int end_index = global_path.nodes_.size() - 1;
  for (int i = global_path.nodes_.size() - 1; i >= 0; i--) {
    if (global_path.nodes_[i].type_ ==
            exploration_path_ns::NodeType::GLOBAL_VIEWPOINT ||
        global_path.nodes_[i].type_ == exploration_path_ns::NodeType::HOME ||
        !viewpoint_manager_->InLocalPlanningHorizon(
            global_path.nodes_[i].position_)) {
      break;
    }
    end_index = i;
  }

  if (!false)
  {
    nav_msgs::Path global_path_trim;
    if (local_path.nodes_.size() >= 2) {
      geometry_msgs::PoseStamped first_pose;
      first_pose.pose.position.x = local_path.nodes_.front().position_.x();
      first_pose.pose.position.y = local_path.nodes_.front().position_.y();
      first_pose.pose.position.z = local_path.nodes_.front().position_.z();
      global_path_trim.poses.push_back(first_pose);
    }

    for (int i = start_index; i <= end_index; i++) {
      geometry_msgs::PoseStamped pose;
      pose.pose.position.x = global_path.nodes_[i].position_.x();
      pose.pose.position.y = global_path.nodes_[i].position_.y();
      pose.pose.position.z = global_path.nodes_[i].position_.z();
      global_path_trim.poses.push_back(pose);
    }
    if (local_path.nodes_.size() >= 2) {
      geometry_msgs::PoseStamped last_pose;
      last_pose.pose.position.x = local_path.nodes_.back().position_.x();
      last_pose.pose.position.y = local_path.nodes_.back().position_.y();
      last_pose.pose.position.z = local_path.nodes_.back().position_.z();
      global_path_trim.poses.push_back(last_pose);
    }
    global_path_trim.header.frame_id = "map";
    global_path_trim.header.stamp = ros::Time::now();
    global_path_publisher_.publish(global_path_trim);
  }
  else
  {
    // publish a blank path
    nav_msgs::Path global_path_trim;
    global_path_trim.header.frame_id = "map";
    global_path_trim.header.stamp = ros::Time::now();
    global_path_publisher_.publish(global_path_trim);
  }
  
  grid_world_->GetVisualizationCloud(grid_world_vis_cloud_->cloud_);
  grid_world_vis_cloud_->Publish();
  grid_world_->GetMarker(grid_world_marker_->marker_);
  grid_world_marker_->Publish();
  nav_msgs::Path full_path = exploration_path_.GetPath();
  full_path.header.frame_id = "map";
  full_path.header.stamp = ros::Time::now();
  exploration_path_publisher_.publish(full_path);
  exploration_path_.GetVisualizationCloud(exploration_path_cloud_->cloud_);
  exploration_path_cloud_->Publish();
  // planning_env_->PublishStackedCloud();
}

void SensorCoveragePlanner3D::LocalPlanning(
    int uncovered_point_num, int uncovered_frontier_point_num,
    const exploration_path_ns::ExplorationPath &global_path,
    exploration_path_ns::ExplorationPath &local_path) {
  misc_utils_ns::Timer local_tsp_timer("Local planning");
  local_tsp_timer.Start();
  if (lookahead_point_update_) {
    local_coverage_planner_->SetLookAheadPoint(lookahead_point_);
  }
  local_path = local_coverage_planner_->SolveLocalCoverageProblem(
      global_path, uncovered_point_num, uncovered_frontier_point_num);
  local_tsp_timer.Stop(false);
}

void SensorCoveragePlanner3D::PublishLocalPlanningVisualization(
    const exploration_path_ns::ExplorationPath &local_path) {
  viewpoint_manager_->GetVisualizationCloud(viewpoint_vis_cloud_->cloud_);
  viewpoint_vis_cloud_->Publish();
  lookahead_point_cloud_->Publish();
  local_coverage_planner_->GetSelectedViewPointVisCloud(
      selected_viewpoint_vis_cloud_->cloud_);
  selected_viewpoint_vis_cloud_->Publish();
  // if finish one room, publish a blank path
  if (!false)
  {
    nav_msgs::Path local_tsp_path = local_path.GetPath();
    local_tsp_path.header.frame_id = "map";
    local_tsp_path.header.stamp = ros::Time::now();
    local_tsp_path_publisher_.publish(local_tsp_path);
  }
  else
  {
    // publish a blank path
    nav_msgs::Path local_tsp_path;
    local_tsp_path.header.frame_id = "map";
    local_tsp_path.header.stamp = ros::Time::now();
    local_tsp_path_publisher_.publish(local_tsp_path);
  }

  // Visualize local planning horizon box
}

void SensorCoveragePlanner3D::PublishFreespaceCloud() {
  viewpoint_manager_->GetFreespaceCloud(freespace_cloud_->cloud_);
  double current_time = ros::Time::now().toSec();
  double delta_time = current_time - start_time_;
  if (delta_time > 20)
  {
    freespace_cloud_->Publish();
  }
}

exploration_path_ns::ExplorationPath
SensorCoveragePlanner3D::ConcatenateGlobalLocalPath(
    const exploration_path_ns::ExplorationPath &global_path,
    const exploration_path_ns::ExplorationPath &local_path) {
  exploration_path_ns::ExplorationPath full_path;
  if (exploration_finished_ && near_home_ && kRushHome) {
    exploration_path_ns::Node node;
    node.position_.x() = robot_position_.x;
    node.position_.y() = robot_position_.y;
    node.position_.z() = robot_position_.z;
    node.type_ = exploration_path_ns::NodeType::ROBOT;
    full_path.nodes_.push_back(node);
    node.position_ = initial_position_;
    node.type_ = exploration_path_ns::NodeType::HOME;
    full_path.nodes_.push_back(node);
    return full_path;
  }

  // if (near_room_1_ && transit_across_room_) {
  //   exploration_path_ns::Node node;
  //   node.position_.x() = robot_position_.x;
  //   node.position_.y() = robot_position_.y;
  //   node.position_.z() = robot_position_.z;
  //   node.type_ = exploration_path_ns::NodeType::ROBOT;
  //   full_path.nodes_.push_back(node);
  //   node.position_ = door_position_;
  //   node.type_ = exploration_path_ns::NodeType::HOME;
  //   full_path.nodes_.push_back(node);
  //   return full_path;
  // }

  double global_path_length = global_path.GetLength();
  double local_path_length = local_path.GetLength();
  if (global_path_length < 3 && local_path_length < 5) {
    return local_path;
  } else {
    full_path = local_path;
    if (local_path.nodes_.front().type_ ==
            exploration_path_ns::NodeType::LOCAL_PATH_END &&
        local_path.nodes_.back().type_ ==
            exploration_path_ns::NodeType::LOCAL_PATH_START) {
      full_path.Reverse();
    } else if (local_path.nodes_.front().type_ ==
                   exploration_path_ns::NodeType::LOCAL_PATH_START &&
               local_path.nodes_.back() == local_path.nodes_.front()) {
      full_path.nodes_.back().type_ =
          exploration_path_ns::NodeType::LOCAL_PATH_END;
    } else if (local_path.nodes_.front().type_ ==
                   exploration_path_ns::NodeType::LOCAL_PATH_END &&
               local_path.nodes_.back() == local_path.nodes_.front()) {
      full_path.nodes_.front().type_ =
          exploration_path_ns::NodeType::LOCAL_PATH_START;
    }
  }
  return full_path;
}

bool SensorCoveragePlanner3D::GetLookAheadPoint(
    const exploration_path_ns::ExplorationPath &local_path,
    const exploration_path_ns::ExplorationPath &global_path,
    Eigen::Vector3d &lookahead_point) {
  Eigen::Vector3d robot_position(robot_position_.x, robot_position_.y,
                                 robot_position_.z);

  // Determine which direction to follow on the global path
  double dist_from_start = 0.0;
  for (int i = 1; i < global_path.nodes_.size(); i++) {
    dist_from_start +=
        (global_path.nodes_[i - 1].position_ - global_path.nodes_[i].position_)
            .norm();
    if (global_path.nodes_[i].type_ ==
        exploration_path_ns::NodeType::GLOBAL_VIEWPOINT) {
      break;
    }
  }

  double dist_from_end = 0.0;
  for (int i = global_path.nodes_.size() - 2; i > 0; i--) {
    dist_from_end +=
        (global_path.nodes_[i + 1].position_ - global_path.nodes_[i].position_)
            .norm();
    if (global_path.nodes_[i].type_ ==
        exploration_path_ns::NodeType::GLOBAL_VIEWPOINT) {
      break;
    }
  }

  bool local_path_too_short = true;
  for (int i = 0; i < local_path.nodes_.size(); i++) {
    double dist_to_robot =
        (robot_position - local_path.nodes_[i].position_).norm();
    if (dist_to_robot > kLookAheadDistance / 5) {
      local_path_too_short = false;
      break;
    }
  }
  if (local_path.GetNodeNum() < 1 || local_path_too_short) {
    if (dist_from_start < dist_from_end) {
      double dist_from_robot = 0.0;
      for (int i = 1; i < global_path.nodes_.size(); i++) {
        dist_from_robot += (global_path.nodes_[i - 1].position_ -
                            global_path.nodes_[i].position_)
                               .norm();
        if (dist_from_robot > kLookAheadDistance / 2) {
          lookahead_point = global_path.nodes_[i].position_;
          break;
        }
      }
    } else {
      double dist_from_robot = 0.0;
      for (int i = global_path.nodes_.size() - 2; i > 0; i--) {
        dist_from_robot += (global_path.nodes_[i + 1].position_ -
                            global_path.nodes_[i].position_)
                               .norm();
        if (dist_from_robot > kLookAheadDistance / 2) {
          lookahead_point = global_path.nodes_[i].position_;
          break;
        }
      }
    }
    return false;
  }

  bool has_lookahead = false;
  bool dir = true;
  int robot_i = 0;
  int lookahead_i = 0;
  for (int i = 0; i < local_path.nodes_.size(); i++) {
    if (local_path.nodes_[i].type_ == exploration_path_ns::NodeType::ROBOT) {
      robot_i = i;
    }
    if (local_path.nodes_[i].type_ ==
        exploration_path_ns::NodeType::LOOKAHEAD_POINT) {
      has_lookahead = true;
      lookahead_i = i;
    }
  }

  if (reset_waypoint_) {
    has_lookahead = false;
  }

  int forward_viewpoint_count = 0;
  int backward_viewpoint_count = 0;

  bool local_loop = false;
  if (local_path.nodes_.front() == local_path.nodes_.back() &&
      local_path.nodes_.front().type_ == exploration_path_ns::NodeType::ROBOT) {
    local_loop = true;
  }

  if (local_loop) {
    robot_i = 0;
  }
  for (int i = robot_i + 1; i < local_path.GetNodeNum(); i++) {
    if (local_path.nodes_[i].type_ ==
        exploration_path_ns::NodeType::LOCAL_VIEWPOINT) {
      forward_viewpoint_count++;
    }
  }
  if (local_loop) {
    robot_i = local_path.nodes_.size() - 1;
  }
  for (int i = robot_i - 1; i >= 0; i--) {
    if (local_path.nodes_[i].type_ ==
        exploration_path_ns::NodeType::LOCAL_VIEWPOINT) {
      backward_viewpoint_count++;
    }
  }

  Eigen::Vector3d forward_lookahead_point = robot_position;
  Eigen::Vector3d backward_lookahead_point = robot_position;

  bool has_forward = false;
  bool has_backward = false;

  if (local_loop) {
    robot_i = 0;
  }
  bool forward_lookahead_point_in_los = true;
  bool backward_lookahead_point_in_los = true;
  double length_from_robot = 0.0;
  for (int i = robot_i + 1; i < local_path.GetNodeNum(); i++) {
    length_from_robot +=
        (local_path.nodes_[i].position_ - local_path.nodes_[i - 1].position_)
            .norm();
    double dist_to_robot =
        (local_path.nodes_[i].position_ - robot_position).norm();
    bool in_line_of_sight = true;
    if (i < local_path.GetNodeNum() - 1) {
      in_line_of_sight = viewpoint_manager_->InCurrentFrameLineOfSight(
          local_path.nodes_[i + 1].position_);
    }
    if ((length_from_robot > kLookAheadDistance ||
         (kUseLineOfSightLookAheadPoint && !in_line_of_sight) ||
         local_path.nodes_[i].type_ ==
             exploration_path_ns::NodeType::LOCAL_VIEWPOINT ||
         local_path.nodes_[i].type_ ==
             exploration_path_ns::NodeType::LOCAL_PATH_START ||
         local_path.nodes_[i].type_ ==
             exploration_path_ns::NodeType::LOCAL_PATH_END ||
         i == local_path.GetNodeNum() - 1))

    {
      if (kUseLineOfSightLookAheadPoint && !in_line_of_sight) {
        forward_lookahead_point_in_los = false;
      }
      forward_lookahead_point = local_path.nodes_[i].position_;
      has_forward = true;
      break;
    }
  }
  if (local_loop) {
    robot_i = local_path.nodes_.size() - 1;
  }
  length_from_robot = 0.0;
  for (int i = robot_i - 1; i >= 0; i--) {
    length_from_robot +=
        (local_path.nodes_[i].position_ - local_path.nodes_[i + 1].position_)
            .norm();
    double dist_to_robot =
        (local_path.nodes_[i].position_ - robot_position).norm();
    bool in_line_of_sight = true;
    if (i > 0) {
      in_line_of_sight = viewpoint_manager_->InCurrentFrameLineOfSight(
          local_path.nodes_[i - 1].position_);
    }
    if ((length_from_robot > kLookAheadDistance ||
         (kUseLineOfSightLookAheadPoint && !in_line_of_sight) ||
         local_path.nodes_[i].type_ ==
             exploration_path_ns::NodeType::LOCAL_VIEWPOINT ||
         local_path.nodes_[i].type_ ==
             exploration_path_ns::NodeType::LOCAL_PATH_START ||
         local_path.nodes_[i].type_ ==
             exploration_path_ns::NodeType::LOCAL_PATH_END ||
         i == 0))

    {
      if (kUseLineOfSightLookAheadPoint && !in_line_of_sight) {
        backward_lookahead_point_in_los = false;
      }
      backward_lookahead_point = local_path.nodes_[i].position_;
      has_backward = true;
      break;
    }
  }

  if (forward_viewpoint_count > 0 && !has_forward) {
    std::cout << "forward viewpoint count > 0 but does not have forward "
                 "lookahead point"
              << std::endl;
    exit(1);
  }
  if (backward_viewpoint_count > 0 && !has_backward) {
    std::cout << "backward viewpoint count > 0 but does not have backward "
                 "lookahead point"
              << std::endl;
    exit(1);
  }

  double dx = lookahead_point_direction_.x();
  double dy = lookahead_point_direction_.y();

  if (reset_waypoint_) {
    reset_waypoint_ = false;
    double lx = 1.0;
    double ly = 0.0;

    dx = cos(robot_yaw_) * lx - sin(robot_yaw_) * ly;
    dy = sin(robot_yaw_) * lx + cos(robot_yaw_) * ly;
  }

  double forward_angle_score = -2;
  double backward_angle_score = -2;
  double lookahead_angle_score = -2;

  double dist_robot_to_lookahead = 0.0;
  if (has_forward) {
    Eigen::Vector3d forward_diff = forward_lookahead_point - robot_position;
    forward_diff.z() = 0.0;
    forward_diff = forward_diff.normalized();
    forward_angle_score = dx * forward_diff.x() + dy * forward_diff.y();
  }
  if (has_backward) {
    Eigen::Vector3d backward_diff = backward_lookahead_point - robot_position;
    backward_diff.z() = 0.0;
    backward_diff = backward_diff.normalized();
    backward_angle_score = dx * backward_diff.x() + dy * backward_diff.y();
  }
  if (has_lookahead) {
    Eigen::Vector3d prev_lookahead_point =
        local_path.nodes_[lookahead_i].position_;
    dist_robot_to_lookahead = (robot_position - prev_lookahead_point).norm();
    Eigen::Vector3d diff = prev_lookahead_point - robot_position;
    diff.z() = 0.0;
    diff = diff.normalized();
    lookahead_angle_score = dx * diff.x() + dy * diff.y();
  }

  lookahead_point_cloud_->cloud_->clear();

  if (forward_viewpoint_count == 0 && backward_viewpoint_count == 0) {
    relocation_ = true;
  } else {
    relocation_ = false;
  }
  if (relocation_) {
    if (use_momentum_ && kUseMomentum) {
      if (forward_angle_score > backward_angle_score) {
        lookahead_point = forward_lookahead_point;
      } else {
        lookahead_point = backward_lookahead_point;
      }
    } else {
      // follow the shorter distance one
      if (dist_from_start < dist_from_end &&
          local_path.nodes_.front().type_ !=
              exploration_path_ns::NodeType::ROBOT) {
        lookahead_point = backward_lookahead_point;
      } else if (dist_from_end < dist_from_start &&
                 local_path.nodes_.back().type_ !=
                     exploration_path_ns::NodeType::ROBOT) {
        lookahead_point = forward_lookahead_point;
      } else {
        lookahead_point = forward_angle_score > backward_angle_score
                              ? forward_lookahead_point
                              : backward_lookahead_point;
      }
    }
  } else if (has_lookahead && lookahead_angle_score > 0 &&
             dist_robot_to_lookahead > kLookAheadDistance / 2 &&
             viewpoint_manager_->InLocalPlanningHorizon(
                 local_path.nodes_[lookahead_i].position_))

  {
    lookahead_point = local_path.nodes_[lookahead_i].position_;
  } else {
    if (forward_angle_score > backward_angle_score) {
      if (forward_viewpoint_count > 0) {
        lookahead_point = forward_lookahead_point;
      } else {
        lookahead_point = backward_lookahead_point;
      }
    } else {
      if (backward_viewpoint_count > 0) {
        lookahead_point = backward_lookahead_point;
      } else {
        lookahead_point = forward_lookahead_point;
      }
    }
  }

  if ((lookahead_point == forward_lookahead_point &&
       !forward_lookahead_point_in_los) ||
      (lookahead_point == backward_lookahead_point &&
       !backward_lookahead_point_in_los)) {
    lookahead_point_in_line_of_sight_ = false;
  } else {
    lookahead_point_in_line_of_sight_ = true;
  }

  lookahead_point_direction_ = lookahead_point - robot_position;
  lookahead_point_direction_.z() = 0.0;
  lookahead_point_direction_.normalize();

  pcl::PointXYZI point;
  point.x = lookahead_point.x();
  point.y = lookahead_point.y();
  point.z = lookahead_point.z();
  point.intensity = 1.0;
  lookahead_point_cloud_->cloud_->points.push_back(point);

  if (has_lookahead) {
    point.x = local_path.nodes_[lookahead_i].position_.x();
    point.y = local_path.nodes_[lookahead_i].position_.y();
    point.z = local_path.nodes_[lookahead_i].position_.z();
    point.intensity = 0;
    lookahead_point_cloud_->cloud_->points.push_back(point);
  }
  return true;
}

void SensorCoveragePlanner3D::PublishWaypoint() {
  geometry_msgs::PointStamped waypoint;
  if (exploration_finished_ && near_home_ && kRushHome) {
    // if the whole environment is explored, and the robot is near home, go back to home
    waypoint.point.x = initial_position_.x();
    waypoint.point.y = initial_position_.y();
    waypoint.point.z = initial_position_.z();
  }
  else if (near_room_1_ && !near_room_2_ && transit_across_room_)
  {
    // If the robot is near the room, go to the door position
    waypoint.point.x = door_position_.x();
    waypoint.point.y = door_position_.y();
    waypoint.point.z = robot_position_.z;
  }
  else if (near_room_2_ && transit_across_room_)
  {
    // If the robot is very near the room, go to the lookahead point
    SendInRoomWaypoint();
    return;
  }
  else if (transit_across_room_ && door_position_.x() > -9999.0 && !near_room_2_)
  {
    waypoint.point.x = door_position_.x();
    waypoint.point.y = door_position_.y();
    waypoint.point.z = robot_position_.z;
  }
  else if ((false || false || ask_found_object_) && !transit_across_room_)
  {
    if (false)
    {
      ROS_INFO("Room finished, waiting for next action");
    }
    if (false)
    {
      ROS_INFO("Accidentally enter a new room");
    }
    if (ask_found_object_)
    {
      ROS_INFO("Found the target object, waiting for next action");
    }
    // If the room is finished, we just send the robot position as the waypoint(not moving)
    waypoint.point.x = robot_position_.x;
    waypoint.point.y = robot_position_.y;
    waypoint.point.z = robot_position_.z;
  }
  else {
    double dx = lookahead_point_.x() - robot_position_.x;
    double dy = lookahead_point_.y() - robot_position_.y;
    double r = sqrt(dx * dx + dy * dy);
    double extend_dist = lookahead_point_in_line_of_sight_
                             ? kExtendWayPointDistanceBig
                             : kExtendWayPointDistanceSmall;
    if (r < extend_dist && kExtendWayPoint) {
      dx = dx / r * extend_dist;
      dy = dy / r * extend_dist;
    }
    waypoint.point.x = dx + robot_position_.x;
    waypoint.point.y = dy + robot_position_.y;
    waypoint.point.z = lookahead_point_.z();
  }
  misc_utils_ns::Publish(&nh_, waypoint_pub_, waypoint,
                         kWorldFrameID);
}

void SensorCoveragePlanner3D::PublishRuntime() {
  local_viewpoint_sampling_runtime_ =
      local_coverage_planner_->GetViewPointSamplingRuntime() / 1000;
  local_path_finding_runtime_ = (local_coverage_planner_->GetFindPathRuntime() +
                                 local_coverage_planner_->GetTSPRuntime()) /
                                1000;

  std_msgs::Int32MultiArray runtime_breakdown_msg;
  runtime_breakdown_msg.data.clear();
  runtime_breakdown_msg.data.push_back(update_representation_runtime_);
  runtime_breakdown_msg.data.push_back(local_viewpoint_sampling_runtime_);
  runtime_breakdown_msg.data.push_back(local_path_finding_runtime_);
  runtime_breakdown_msg.data.push_back(global_planning_runtime_);
  runtime_breakdown_msg.data.push_back(trajectory_optimization_runtime_);
  runtime_breakdown_msg.data.push_back(overall_runtime_);
  runtime_breakdown_pub_.publish(runtime_breakdown_msg);

  float runtime = 0;
  if (!exploration_finished_ && kNoExplorationReturnHome) {
    for (int i = 0; i < runtime_breakdown_msg.data.size() - 1; i++) {
      runtime += runtime_breakdown_msg.data[i];
    }
  }

  std_msgs::Float32 runtime_msg;
  runtime_msg.data = runtime / 1000.0;
  runtime_pub_.publish(runtime_msg);
}

double SensorCoveragePlanner3D::GetRobotToHomeDistance() {
  Eigen::Vector3d robot_position(robot_position_.x, robot_position_.y,
                                 robot_position_.z);
  return (robot_position - initial_position_).norm();
}

double SensorCoveragePlanner3D::GetRobotToRoomDistance()
{
  Eigen::Vector3d robot_position(robot_position_.x, robot_position_.y,
                                 robot_position_.z);
  double euler_length = (robot_position - door_position_).norm();
  double exploration_path_length = exploration_path_.GetLength() / 2.0; // because the path is a loop
  return std::max(euler_length, exploration_path_length);
}

void SensorCoveragePlanner3D::GetToRoomState(bool &at_room, bool &near_room_1, bool &near_room_2) 
{
  if (!transit_across_room_) {
    at_room = false;
    near_room_1 = false;
    near_room_2 = false;
    return;
  }
  else
  {
    at_room_ = (current_room_id_ == end_room_id_) && (end_room_id_ != -1);
    Eigen::Vector3d robot_position(robot_position_.x, robot_position_.y,
                                   robot_position_.z);
    double euler_length = (robot_position - door_position_).norm();
    double exploration_path_length = exploration_path_.GetLength() / 2.0;
    const double robot_to_room_dist = std::max(euler_length, exploration_path_length);
    ROS_INFO("Robot to room distance: %f, Exploration path length: %f",
             euler_length, exploration_path_length);
    near_room_1_ = (robot_to_room_dist < kRushRoomDist_1) || at_room_;
    near_room_2_ = (robot_to_room_dist < kRushRoomDist_2) || at_room_;
    if (near_room_2_)
    {
      if (door_normal_.norm() > 0.1)
      {
        // calculate the relative angle of the robot orientation and the room normal
        // 判断机器人位置和房门口连线的方向
        Eigen::Vector3d direction_1(cos(robot_yaw_), sin(robot_yaw_), 0.0);
        Eigen::Vector3d direction_2 = door_position_ - Eigen::Vector3d(robot_position_.x, robot_position_.y, robot_position_.z);
        direction_2.z() = 0.0;
        direction_2.normalize();

        double angle_1 = acos(direction_1.dot(door_normal_));
        double angle_2 = acos(direction_2.dot(door_normal_));
        ROS_ERROR("Angle 1: %f, Angle 2: %f", angle_1 / M_PI * 180.0, angle_2 / M_PI * 180.0);
        bool yaw_flag = angle_1 < M_PI / 180.0 * 45.0;
        bool direction_flag = angle_2 < M_PI / 180.0 * 45.0;
        near_room_2_ = (near_room_2_ && yaw_flag) || direction_flag || at_room_;
        if (robot_to_room_dist < kRushRoomDist_2 * 0.75)
        {
          near_room_2_ = true;
        }
      }
    }
    ROS_INFO("!!!!!!!!!!!!!!Near room 1: %d, Near room 2: %d, At room: %d, Current room: %d, Target room: %d",
                near_room_1_, near_room_2_, at_room_, current_room_id_, end_room_id_);
  }
}

void SensorCoveragePlanner3D::PublishExplorationState() {
  std_msgs::Bool exploration_finished_msg;
  exploration_finished_msg.data = exploration_finished_;
  exploration_finish_pub_.publish(exploration_finished_msg);
}

void SensorCoveragePlanner3D::PrintExplorationStatus(std::string status,
                                                     bool clear_last_line) {
  if (clear_last_line) {
    printf(cursup);
    printf(cursclean);
    printf(cursup);
    printf(cursclean);
  }
  std::cout << std::endl << "\033[1;32m" << status << "\033[0m" << std::endl;
}

void SensorCoveragePlanner3D::CountDirectionChange() {
  Eigen::Vector3d current_moving_direction_ =
      Eigen::Vector3d(robot_position_.x, robot_position_.y, robot_position_.z) -
      Eigen::Vector3d(last_robot_position_.x, last_robot_position_.y,
                      last_robot_position_.z);

  if (current_moving_direction_.norm() > 0.5) {
    if (moving_direction_.dot(current_moving_direction_) < 0) {
      direction_change_count_++;
      direction_no_change_count_ = 0;
      if (direction_change_count_ > kDirectionChangeCounterThr) {
        if (!use_momentum_) {
          momentum_activation_count_++;
        }
        use_momentum_ = true;
      }
    } else {
      direction_no_change_count_++;
      if (direction_no_change_count_ > kDirectionNoChangeCounterThr) {
        direction_change_count_ = 0;
        use_momentum_ = false;
      }
    }
    moving_direction_ = current_moving_direction_;
  }
  last_robot_position_ = robot_position_;

  std_msgs::Int32 momentum_activation_count_msg;
  momentum_activation_count_msg.data = momentum_activation_count_;
  momentum_activation_count_pub_.publish(momentum_activation_count_msg);
}

void SensorCoveragePlanner3D::execute() {
  if (!kAutoStart && !start_exploration_) {
    ROS_INFO("Waiting for start signal");
    return;
  }
  Timer overall_processing_timer("overall processing");
  update_representation_runtime_ = 0;
  local_viewpoint_sampling_runtime_ = 0;
  local_path_finding_runtime_ = 0;
  global_planning_runtime_ = 0;
  trajectory_optimization_runtime_ = 0;
  overall_runtime_ = 0;

  if (!initialized_) {
    SendInitialWaypoint();
    start_time_ = ros::Time::now().toSec();
    if(start_time_ == 0.0){
      ROS_ERROR("Start time is zero, time source (use_time_time) not set correctly. Exiting...");
      exit(1);
    }
    global_direction_switch_time_ = ros::Time::now().toSec();
    initialized_ = true;
    return;
  }

  ProcessObjectNodes();
  CheckObjectFound();
  if (dynamic_environment_)
  {
    CheckAnchorObjectFound();
  }

  if (tmp_flag_)
  {
    tmp_flag_ = false;
    viewpoint_manager_->ResetViewPointCoverage();
    ROS_ERROR("Reset the viewpoint coverage");
  }

  overall_processing_timer.Start();
  if (keypose_cloud_update_) {
    keypose_cloud_update_ = false;
    UpdateRoomLabel();
    SetCurrentRoomId();

    // const bool room_transit_canceled = MarkDepartingRoomUnfinishedIfNeeded();

    // -------- Transit across rooms --------
    if (transit_across_room_ && !at_room_)
    {
      geometry_msgs::PointStamped::Ptr geomsg(
          new geometry_msgs::PointStamped());
      geomsg->header.frame_id = kWorldFrameID;
      geomsg->header.stamp = ros::Time::now();
      geomsg->point.x = goal_position_.x;
      geomsg->point.y = goal_position_.y;
      geomsg->point.z = goal_position_.z;
      GoalPointCallback(geomsg);
      SetStartAndEndRoomId();
    }
    if (at_room_)
    {
      room_guide_counter_++;
      reset_waypoint_ = true;

      // let the local coverage planner first starting the sampling
      local_coverage_planner_->SetTransitAcrossRoom(false);
      viewpoint_manager_->SetTransitAcrossRoom(false);
      if (room_guide_counter_ % 3 == 0)
      {
        ROS_INFO("Arrived at the room, waiting for next action");
        room_guide_counter_ = 0;
        stayed_in_room_counter_ = 0;
        ResetRoomInfo();
      }
    }

    CountDirectionChange();

    misc_utils_ns::Timer update_representation_timer("update representation");
    update_representation_timer.Start();
    UpdateObjectVisibility();

    // Update grid world
    UpdateGlobalRepresentation();
    UpdateViewpointRep();
    // Draw the current viewpoint representation's room index
    PublishViewpointRoomIdMarkers();

    if (add_viewpoint_rep_)
    {
      UpdateViewpointObjectVisibility();
      add_viewpoint_rep_ = false;
    }

    // Update the visibility markers after updating the object visibility
    CreateVisibilityMarkers();
    
    int viewpoint_candidate_count = UpdateViewPoints();
    if (viewpoint_candidate_count == 0) {
      ROS_WARN("Cannot get candidate viewpoints, skipping this round");
      return;
    }

    CheckDoorCloudInRange();
    UpdateKeyposeGraph();

    int uncovered_point_num = 0;
    int uncovered_frontier_point_num = 0;
    if (!exploration_finished_ || !kNoExplorationReturnHome) {
      UpdateViewPointCoverage();
      UpdateCoveredAreas(uncovered_point_num, uncovered_frontier_point_num);
    } else {
      viewpoint_manager_->ResetViewPointCoverage();
    }

    update_representation_timer.Stop(false);
    update_representation_runtime_ +=
        update_representation_timer.GetDuration("ms");

    // Global TSP
    std::vector<int> global_cell_tsp_order;
    exploration_path_ns::ExplorationPath global_path;
    GlobalPlanning(global_cell_tsp_order, global_path);

    // Local TSP
    exploration_path_ns::ExplorationPath local_path;
    LocalPlanning(uncovered_point_num, uncovered_frontier_point_num,
                  global_path, local_path);

    near_home_ = GetRobotToHomeDistance() < kRushHomeDist;
    at_home_ = GetRobotToHomeDistance() < kAtHomeDistThreshold;

    double current_time = ros::Time::now().toSec();
    double delta_time = current_time - start_time_;

    if (grid_world_->IsReturningHome() &&
        local_coverage_planner_->IsLocalCoverageComplete() &&
        (current_time - start_time_) > 5) {
      if (!exploration_finished_) {
        PrintExplorationStatus("Exploration completed, returning home", false);
      }
      exploration_finished_ = true;
    }

    if (exploration_finished_ && at_home_ && !stopped_) {
      PrintExplorationStatus("Return home completed", false);
      stopped_ = true;
    }

    // Handle room finishing and changing
    // if (suppress_room_transit_after_cancel_) {
    //   if (!local_coverage_planner_->IsLocalCoverageComplete() ||
    //       !grid_world_->HasExploringCellsInRoom(suppress_room_transit_room_id_)) {
    //     suppress_room_transit_after_cancel_ = false;
    //     suppress_room_transit_room_id_ = -1;
    //   }
    // }

    if (current_room_id_ != -1)
    {
      if (!representation_->HasRoomNode(current_room_id_)) {
        ROS_WARN("Current room with id %d does not exist in representation, reset to -1", current_room_id_);
        current_room_id_ = -1;
      }
      else {
        auto &current_room = representation_->GetRoomNode(current_room_id_);
        if ((grid_world_->IsRoomFinished() && local_coverage_planner_->IsLocalCoverageComplete() &&
               !transit_across_room_) ||
            (current_room.area_ < 10.0 && stayed_in_room_counter_ > 20)) {
          if (representation_->HasRoomNode(current_room_id_)) {
            representation_->GetRoomNode(current_room_id_).SetIsCovered(true);
          }
          if (!has_candidate_room_position_) {
            SelectNearestUnexploredRoom();
          }
          GetAnswer();
        }
        else {
          if (representation_->HasRoomNode(current_room_id_) && !transit_across_room_) {
            representation_->GetRoomNode(current_room_id_).SetIsCovered(false);
          }
        }
      }
    }

    exploration_path_ = ConcatenateGlobalLocalPath(global_path, local_path);
    GetToRoomState(at_room_, near_room_1_, near_room_2_);

    PublishExplorationState();

    lookahead_point_update_ = GetLookAheadPoint(exploration_path_, global_path, lookahead_point_);
    PublishWaypoint();

    overall_processing_timer.Stop(false);
    overall_runtime_ = overall_processing_timer.GetDuration("ms");
 
    visualizer_->GetGlobalSubspaceMarker(grid_world_, global_cell_tsp_order);
    Eigen::Vector3d viewpoint_origin = viewpoint_manager_->GetOrigin();
    visualizer_->GetLocalPlanningHorizonMarker(viewpoint_origin.x(), viewpoint_origin.y(), robot_position_.z);
    visualizer_->PublishMarkers();
    
    PublishFreespaceCloud();

    PublishLocalPlanningVisualization(local_path);
    PublishGlobalPlanningVisualization(global_path, local_path);
    PublishRoomTypeVisualization();
    PublishObjectNodeMarkers();
    PublishRuntime();

    stayed_in_room_counter_++;
  }
}

void SensorCoveragePlanner3D::UpdateViewpointRep(){
  if (!initialized_) {
    ROS_ERROR("Planner not initialized, cannot update viewpoint representation");
    return;
  }

  planning_env_->GetUpdatedVoxelInds(current_obs_voxel_inds_);

  std::vector<int> intersection_voxel_inds;
  // get the intersection of current_obs_voxel_inds_ and previous_obs_voxel_inds_
  misc_utils_ns::SetIntersection(current_obs_voxel_inds_,
                                 previous_obs_voxel_inds_,
                                 intersection_voxel_inds);
  int intersection_voxel_num = intersection_voxel_inds.size();
  int current_obs_voxel_num = current_obs_voxel_inds_.size();
  int previous_obs_voxel_num = previous_obs_voxel_inds_.size();
  double ratio = intersection_voxel_num / (double)current_obs_voxel_num;

  if ((intersection_voxel_num < rep_threshold_voxel_num_ && ratio < rep_threshold_))
  {
    // If the intersection is less than 20% of the current obs voxel number,
    // we update the viewpoint representation
    ROS_INFO("Intersection voxel number is low, updating viewpoint representation.");
    add_viewpoint_rep_ = true;
  }
  // ROS_ERROR("Object score: %f.", obj_score_);
  if (obj_score_ > 4.0)
  {
    ROS_INFO("Object score is high, adding viewpoint representation.");
    add_viewpoint_rep_ = true;
  }

  if (add_viewpoint_rep_) 
  {
    // ROS_INFO("Intersection voxel number: %d, Current obs voxel number: %d, Pre obs voxel number: %d",
    //             intersection_voxel_num, current_obs_voxel_num, previous_obs_voxel_num);
    // ROS_INFO("Intersection ratio: %f, Threshold : %d", ratio, rep_threshold_voxel_num_);
    // ROS_INFO("Updating viewpoint representation.");
    // add_viewpoint_rep_ = false;
    pcl::PointCloud<pcl::PointXYZI>::Ptr covered_cloud = planning_env_->GetUpdatedCloudInRange();
    int prev_size = representation_->GetViewPointReps().size();
    curr_viewpoint_rep_node_ind = representation_->AddViewPointRep(robot_position_, keypose_cloud_->cloud_, covered_cloud, viewpoint_rep_msg_.header.stamp);
    int curr_size = representation_->GetViewPointReps().size();
    representation_->GetViewPointRepNode(curr_viewpoint_rep_node_ind).SetRoomId(current_room_id_);
    geometry_msgs::Point current_viewpoint_rep_node_pos = representation_->GetViewPointRepNodePos(curr_viewpoint_rep_node_ind);

    // publish viewpoint_rep_header_ only if we are actually adding a new viewpoint representation on rviz
    if (prev_size != curr_size) 
    {
      viewpoint_rep_msg_.viewpoint_id = curr_viewpoint_rep_node_ind;
      viewpoint_rep_pub_.publish(viewpoint_rep_msg_);
      // Update the viewpoint representation cloud
      Eigen::Vector3d origin(current_viewpoint_rep_node_pos.x,
                             current_viewpoint_rep_node_pos.y,
                             current_viewpoint_rep_node_pos.z);
      planning_env_->UpdateCoveredVoxels(origin);
      planning_env_->GetCurrentObsVoxelInds(previous_obs_voxel_inds_);

      representation_->GetLatestObjectNodeIndicesMutable().clear();
    }
  }
  viewpoint_rep_vis_cloud_->cloud_ = representation_->GetViewPointRepCloud();
  viewpoint_rep_vis_cloud_->Publish();
  covered_points_all_->cloud_ = representation_->GetCoveredPointsAllCloud();
  covered_points_all_->Publish();
}

void SensorCoveragePlanner3D::UpdateRoomLabel()
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr covered_cloud = planning_env_->GetUpdatedCloudInRange();
  // ctreat a std::array with the same size as the room_nodes_.size
  // std::vector<int> room_counts(representation_->GetRoomNodeCount(), 0);
  std::unordered_map<int, int> room_counts;
  std::unordered_map<int, pcl::PointCloud<pcl::PointXYZI>> room_cloud_in_range;
  std::unordered_map<int, Eigen::Vector3f> room_centers;
  for (auto &id_to_room_node : representation_->GetRoomNodesMapMutable())
  {
    int room_id = id_to_room_node.first;
    room_counts[room_id] = 0;
    room_cloud_in_range[room_id] = pcl::PointCloud<pcl::PointXYZI>();
    room_centers[room_id] = Eigen::Vector3f(0.0, 0.0, 0.0);
  }
  for (const auto &point : covered_cloud->points)
  {
    Eigen::Vector3f point_pos(point.x, point.y, point.z);
    Eigen::Vector3i point_voxel_ind = misc_utils_ns::point_to_voxel(point_pos, shift_, 1.0 / room_resolution_);
    int room_id = room_mask_.at<int>(point_voxel_ind.x(), point_voxel_ind.y());
    if (representation_->HasRoomNode(room_id))
    {
      room_counts[room_id]++;
      room_cloud_in_range[room_id].points.push_back(point);
      room_centers[room_id] += Eigen::Vector3f(point.x, point.y, point.z);
    }
  }
  std::vector<int> labled_rooms = {};
  for (auto &id_to_room_node : representation_->GetRoomNodesMapMutable())
  {
    int room_id = id_to_room_node.first;
    auto &room_node = id_to_room_node.second;
    // First deal with the unlabeled rooms
    if (room_counts[room_id] == 0)
    {
      continue;
    }
    else if (representation_->GetRoomNode(room_id).IsLabeled())
    {
      labled_rooms.push_back(room_id);
      continue;
    }
    else
    {
      room_centers[room_id] /= room_counts[room_id];
      representation_->GetRoomNode(room_id).SetVoxelNum(room_counts[room_id]);
      representation_->GetRoomNode(room_id).SetIsLabeled(true);

      pcl::PointCloud<pcl::PointXYZI>::Ptr room_cloud_tmp(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::PointXYZI room_center;
      room_center.x = room_centers[room_id].x();
      room_center.y = room_centers[room_id].y();
      room_center.z = robot_position_.z; // use the robot z position as the room center z
      room_center.intensity = 10.0;
      pcl::copyPointCloud((room_cloud_in_range[room_id]), *room_cloud_tmp);
      room_cloud_tmp->push_back(room_center);

      // publish it with room_cloud_pub_
      sensor_msgs::PointCloud2 room_cloud_msg;
      pcl::toROSMsg(*room_cloud_tmp, room_cloud_msg);
      room_cloud_msg.header.frame_id = kWorldFrameID;
      room_cloud_msg.header.stamp = ros::Time::now();
      room_cloud_pub_.publish(room_cloud_msg);

      // pcl::copyPointCloud(*(keypose_cloud_->cloud_),
      //                     *door_cloud);
      float lidarRoll = 0, lidarPitch = 0, lidarYaw = 0;
      float lidarX = 0, lidarY = 0, lidarZ = 0;
      GetPoseAtTime(imageTime, lidarX, lidarY, lidarZ, lidarRoll, lidarPitch, lidarYaw);
      cv::Mat camera_image_tmp = camera_image_.clone();
      cv::Mat cropped_img = project_pcl_to_image(room_cloud_tmp, lidarX, lidarY, lidarZ,
                                          lidarRoll, lidarPitch, lidarYaw,
                                          camera_image_tmp, room_center, room_id);
      // save the current camera_image_
      std::string image_path = "debug/" + std::to_string(room_id) + ".png";
      std::string mask_path = "debug/" + std::to_string(room_id) + "_mask.png";
      // cv::imwrite(image_path, cropped_img);
      // cv::imwrite(mask_path, room_node.room_mask_);
      room_node.SetImage(cropped_img);
      // ROS_INFO("Saved room image to %s", image_path.c_str());
      
      geometry_msgs::Point anchor_point;
      anchor_point.x = room_center.x;
      anchor_point.y = room_center.y;
      anchor_point.z = robot_position_.z; // use the robot z position as the anchor point z
      // room_node.anchor_point_ = anchor_point;
      room_node.SetAnchorPoint(anchor_point);
      room_node.SetLastArea(room_node.area_);

      tare_planner::RoomType room_type_msg;
      room_type_msg.header.frame_id = kWorldFrameID;
      room_type_msg.header.stamp = ros::Time::now();
      room_type_msg.anchor_point = anchor_point;
      room_type_msg.room_id = room_id;
      room_type_msg.in_room = (room_id == current_room_id_);
      auto img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cropped_img).toImageMsg();
      auto room_mask_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", room_node.room_mask_).toImageMsg();
      room_type_msg.image = *img_msg; // 注意是解引用
      room_type_msg.room_mask = *room_mask_msg;
      room_type_msg.room_type = "";
      room_type_msg.voxel_num = room_counts[room_id];

      room_type_pub_.publish(room_type_msg);
      // -------- Early Stop 1 --------
      if (room_counts[room_id] > 100 && room_node.GetIsAsked() > 0 && !room_node.IsCovered() && !room_node.IsVisited() && !transit_across_room_)
      {
        
      }
    }
  }
  for(int room_id : labled_rooms)
  {
    if (!representation_->HasRoomNode(room_id)) {
      ROS_WARN("Room with id %d does not exist in representation, skip", room_id);
      continue;
    }
    auto &room_node = representation_->GetRoomNode(room_id);
    if (room_counts[room_id] - representation_->GetRoomNode(room_id).GetVoxelNum() > 20 ||
        room_node.area_ - room_node.last_area_ > 5.0)
    // if (room_counts[room_id] - representation_->GetRoomNode(room_id).voxel_num_ > 40)
    {
      bool flag1 = (room_counts[room_id] - representation_->GetRoomNode(room_id).GetVoxelNum() > 20);
      bool flag2 = (room_node.area_ - room_node.last_area_ > 5.0);

      room_centers[room_id] /= room_counts[room_id];
      representation_->GetRoomNode(room_id).SetVoxelNum(room_counts[room_id]);
      representation_->GetRoomNode(room_id).SetIsLabeled(true);

      pcl::PointCloud<pcl::PointXYZI>::Ptr room_cloud_tmp(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::PointXYZI room_center;
      room_center.x = room_centers[room_id].x();
      room_center.y = room_centers[room_id].y();
      room_center.z = robot_position_.z; // use the robot z position as the room center z
      room_center.intensity = 10.0;
      pcl::copyPointCloud((room_cloud_in_range[room_id]), *room_cloud_tmp);
      room_cloud_tmp->push_back(room_center);

      // publish it with room_cloud_pub_
      sensor_msgs::PointCloud2 room_cloud_msg;
      pcl::toROSMsg(*room_cloud_tmp, room_cloud_msg);
      room_cloud_msg.header.frame_id = kWorldFrameID;
      room_cloud_msg.header.stamp = ros::Time::now();
      room_cloud_pub_.publish(room_cloud_msg);

      cv::Mat cropped_img;
      geometry_msgs::Point anchor_point;
      if (flag1)
      {
        float lidarRoll = 0, lidarPitch = 0, lidarYaw = 0;
        float lidarX = 0, lidarY = 0, lidarZ = 0;
        GetPoseAtTime(imageTime, lidarX, lidarY, lidarZ, lidarRoll, lidarPitch, lidarYaw);
        cv::Mat camera_image_tmp = camera_image_.clone();
        cropped_img = project_pcl_to_image(room_cloud_tmp, lidarX, lidarY, lidarZ,
                                          lidarRoll, lidarPitch, lidarYaw,
                                          camera_image_tmp, room_center, room_id);
        // save the current camera_image_
        std::string image_path = "debug/" + std::to_string(room_id) + ".png";
        // cv::imwrite(image_path, cropped_img);
        room_node.SetImage(cropped_img);

        // choose the first point in the room_cloud_in_range[room_id - 1] as the anchor point
        anchor_point.x = room_center.x;
        anchor_point.y = room_center.y;
        anchor_point.z = robot_position_.z; // use the robot z position as the anchor point z
        room_node.SetAnchorPoint(anchor_point);
      }
      else
      {
        cropped_img = room_node.GetImage();
        anchor_point = room_node.GetAnchorPoint();
      }
      std::string mask_path = "debug/" + std::to_string(room_id) + "_mask.png";
      // cv::imwrite(mask_path, room_node.room_mask_);

      room_node.SetLastArea(room_node.area_);

      tare_planner::RoomType room_type_msg;
      room_type_msg.anchor_point = anchor_point;
      room_type_msg.room_id = room_id;
      room_type_msg.in_room = (room_id == current_room_id_);
      auto img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cropped_img).toImageMsg();
      auto room_mask_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", room_node.room_mask_).toImageMsg();
      room_type_msg.image = *img_msg; // 注意是解引用
      room_type_msg.room_mask = *room_mask_msg;
      room_type_msg.room_type = "";
      room_type_msg.voxel_num = room_counts[room_id];

      room_type_pub_.publish(room_type_msg);
      // -------- Early Stop 1 --------
      if (room_counts[room_id] > 200 && room_node.GetIsAsked() > 0 && flag1 && !room_node.IsCovered() && !room_node.IsVisited() && !transit_across_room_)
      {
        
      }
    }
  }
  if (false)
  {
    
  }
}

void SensorCoveragePlanner3D::GetPoseAtTime(double imageTime, float &lidarX, float &lidarY, float &lidarZ, float &lidarRoll, float &lidarPitch, float &lidarYaw)
{
  while (odomFrontIDPointer != odomLastIDPointer)
  {
    if (odomTimeStack[odomFrontIDPointer] > imageTime)
    {
      break;
    }
    odomFrontIDPointer = (odomFrontIDPointer + 1) % 400;
  }
  if (odomTimeStack[odomFrontIDPointer] < imageTime)
  {
    lidarX = lidarXStack[odomFrontIDPointer];
    lidarY = lidarYStack[odomFrontIDPointer];
    lidarZ = lidarZStack[odomFrontIDPointer];
    lidarRoll = lidarRollStack[odomFrontIDPointer];
    lidarPitch = lidarPitchStack[odomFrontIDPointer];
    lidarYaw = lidarYawStack[odomFrontIDPointer];
  }
  else
  {
    int odomBackIDPointer = (odomFrontIDPointer - 1) % 400;
    float ratioFront = (imageTime - odomTimeStack[odomBackIDPointer]) / (odomTimeStack[odomFrontIDPointer] - odomTimeStack[odomBackIDPointer]);
    float ratioBack = (odomTimeStack[odomFrontIDPointer] - imageTime) / (odomTimeStack[odomFrontIDPointer] - odomTimeStack[odomBackIDPointer]);

    if (lidarYawStack[odomFrontIDPointer] - lidarYawStack[odomBackIDPointer] > PI)
    {
      lidarYawStack[odomBackIDPointer] += 2 * PI;
    }
    else if (lidarYawStack[odomFrontIDPointer] - lidarYawStack[odomBackIDPointer] < -PI)
    {
      lidarYawStack[odomBackIDPointer] -= 2 * PI;
    }

    lidarX = lidarXStack[odomFrontIDPointer] * ratioFront + lidarXStack[odomBackIDPointer] * ratioBack;
    lidarY = lidarYStack[odomFrontIDPointer] * ratioFront + lidarYStack[odomBackIDPointer] * ratioBack;
    lidarZ = lidarZStack[odomFrontIDPointer] * ratioFront + lidarZStack[odomBackIDPointer] * ratioBack;
    lidarRoll = lidarRollStack[odomFrontIDPointer] * ratioFront + lidarRollStack[odomBackIDPointer] * ratioBack;
    lidarPitch = lidarPitchStack[odomFrontIDPointer] * ratioFront + lidarPitchStack[odomBackIDPointer] * ratioBack;
    lidarYaw = lidarYawStack[odomFrontIDPointer] * ratioFront + lidarYawStack[odomBackIDPointer] * ratioBack;
  }
}

// Publish the room type visualization
void SensorCoveragePlanner3D::PublishRoomTypeVisualization()
{
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker clear_marker;
  clear_marker.header.frame_id = kWorldFrameID;
  clear_marker.header.stamp = ros::Time::now();
  clear_marker.ns = "room_type";
  clear_marker.id = 0;
  clear_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  clear_marker.action = visualization_msgs::Marker::DELETEALL;
  marker_array.markers.push_back(clear_marker);
  for (const auto &id_room_node_pair : representation_->GetRoomNodesMap())
  {
    const representation_ns::RoomNodeRep &room_node = id_room_node_pair.second;
    if (room_node.IsLabeled())
    {
      // ROS_INFO("Room %d is labeled with type %s", room_node.show_id_, room_node.label_.c_str());
      visualization_msgs::Marker marker;
      marker.header.frame_id = kWorldFrameID;
      marker.header.stamp = ros::Time::now();
      marker.ns = "room_type";
      marker.id = room_node.show_id_;
      marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      marker.action = visualization_msgs::Marker::ADD;
      // marker.pose.position.x = room_node.centroid_.x();
      // marker.pose.position.y = room_node.centroid_.y();
      // marker.pose.position.z = room_node.centroid_.z();
      marker.pose.position.x = room_node.anchor_point_.x;
      marker.pose.position.y = room_node.anchor_point_.y;
      marker.pose.position.z = room_node.anchor_point_.z;
      marker.pose.orientation.w = 0.65;
      marker.scale.z = 1.0;
      marker.color.a = 1.0;
      Eigen::Vector3d color = misc_utils_ns::idToColor(room_node.GetId());
      marker.color.b = color[0] / 255.0;
      marker.color.g = color[1] / 255.0;
      marker.color.r = color[2] / 255.0;
      // text is room_id + " " + room_node.label_
      // marker.text = std::to_string(room_node.show_id_) + "(" + std::to_string(room_node.GetId()) + ")" + room_node.label_;
      std::string label = room_node.GetRoomLabel();
      marker.text = std::to_string(room_node.GetId()) + " " + label;
      marker_array.markers.push_back(marker);
    }
    // else
    // {
    //   visualization_msgs::Marker marker;
    //   marker.header.frame_id = kWorldFrameID;
    //   marker.header.stamp = ros::Time::now();
    //   marker.ns = "room_type";
    //   marker.id = room_node.show_id_;
    //   marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    //   marker.action = visualization_msgs::Marker::ADD;
    //   marker.pose.position.x = room_node.centroid_.x();
    //   marker.pose.position.y = room_node.centroid_.y();
    //   marker.pose.position.z = room_node.centroid_.z();
    //   // marker.pose.position.x = room_node.anchor_point_.x;
    //   // marker.pose.position.y = room_node.anchor_point_.y;
    //   // marker.pose.position.z = room_node.anchor_point_.z;
    //   marker.pose.orientation.w = 0.65;
    //   marker.scale.z = 1.0;
    //   marker.color.a = 1.0;
    //   Eigen::Vector3d color = misc_utils_ns::idToColor(room_node.GetId());
    //   marker.color.b = color[0] / 255.0;
    //   marker.color.g = color[1] / 255.0;
    //   marker.color.r = color[2] / 255.0;
    //   marker.text = std::to_string(room_node.GetId());
    //   marker_array.markers.push_back(marker);
    // }
  }
  room_type_vis_pub_.publish(marker_array);
}

cv::Mat SensorCoveragePlanner3D::project_pcl_to_image(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_w,
    float &lidarX, float &lidarY, float &lidarZ, float &lidarRoll, float &lidarPitch, float &lidarYaw,
    cv::Mat &image, pcl::PointXYZI &room_center, int &room_id)
{
  cv::Mat image_projected = image.clone();
  cv::Mat rotated_image = image.clone();
  const float PI = 3.1415926f;
  const float camX = -0.12f, camY = -0.075f, camZ = 0.265f;
  const float camRoll = -1.5707963f, camPitch = 0.0f, camYaw = -1.5707963f;
  const int imageWidth = 1920;
  const int imageHeight = 640;
  const float minRange = 0.5f, maxRange = 10.0f;

  int imagePixelNum = imageWidth * imageHeight;

  float sinCamRoll = sin(camRoll);
  float cosCamRoll = cos(camRoll);
  float sinCamPitch = sin(camPitch);
  float cosCamPitch = cos(camPitch);
  float sinCamYaw = sin(camYaw);
  float cosCamYaw = cos(camYaw);

  float sinLidarRoll = sin(lidarRoll);
  float cosLidarRoll = cos(lidarRoll);
  float sinLidarPitch = sin(lidarPitch);
  float cosLidarPitch = cos(lidarPitch);
  float sinLidarYaw = sin(lidarYaw);
  float cosLidarYaw = cos(lidarYaw);

  int cloud_wSize = cloud_w->points.size();

  std::vector<int> hori_coords; // 收集所有有效的horiPixelID
  int hori_coord_room_center = -1;

  for (int i = 0; i <= cloud_wSize; i++)
  {
    float x1, y1, z1;
    if (i < cloud_wSize)
    {
      x1 = cloud_w->points[i].x - lidarX;
      y1 = cloud_w->points[i].y - lidarY;
      z1 = cloud_w->points[i].z - lidarZ;
    }
    else
    {
      x1 = room_center.x - lidarX;
      y1 = room_center.y - lidarY;
      z1 = room_center.z - lidarZ;
    }

    float x2 = x1 * cosLidarYaw + y1 * sinLidarYaw;
    float y2 = -x1 * sinLidarYaw + y1 * cosLidarYaw;
    float z2 = z1;

    float x3 = x2 * cosLidarPitch - z2 * sinLidarPitch;
    float y3 = y2;
    float z3 = x2 * sinLidarPitch + z2 * cosLidarPitch;

    float x4 = x3;
    float y4 = y3 * cosLidarRoll + z3 * sinLidarRoll;
    float z4 = -y3 * sinLidarRoll + z3 * cosLidarRoll;

    float x5 = x4 - camX;
    float y5 = y4 - camY;
    float z5 = z4 - camZ;

    float x6 = x5 * cosCamYaw + y5 * sinCamYaw;
    float y6 = -x5 * sinCamYaw + y5 * cosCamYaw;
    float z6 = z5;

    float x7 = x6 * cosCamPitch - z6 * sinCamPitch;
    float y7 = y6;
    float z7 = x6 * sinCamPitch + z6 * cosCamPitch;

    float x8 = x7;
    float y8 = y7 * cosCamRoll + z7 * sinCamRoll;
    float z8 = -y7 * sinCamRoll + z7 * cosCamRoll;

    int horiPixelID = -1, vertPixelID = -1;
    float horiDis = sqrt(x8 * x8 + z8 * z8);

    horiPixelID = imageWidth / (2 * PI) * atan2(x8, z8) + imageWidth / 2 + 1;
    vertPixelID = imageWidth / (2 * PI) * atan(y8 / horiDis) + imageHeight / 2 + 1;

    int pixelVal = 255 * (horiDis - minRange) / (maxRange - minRange);

    if (i < cloud_wSize)
    {
      if (horiPixelID >= 0 && horiPixelID <= imageWidth - 1 && vertPixelID >= 0 && vertPixelID <= imageHeight - 1)
      {
        for (int du = -1; du <= 1; ++du)
        {
          for (int dv = -1; dv <= 1; ++dv)
          {
            int uu = std::min(imageWidth - 1, std::max(0, horiPixelID + du));
            int vv = std::min(imageHeight - 1, std::max(0, vertPixelID + dv));
            int idx = vv * imageWidth + uu;
            {
              image_projected.at<cv::Vec3b>(vv, uu)[0] = pixelVal;
              image_projected.at<cv::Vec3b>(vv, uu)[1] = 255 - pixelVal;
              image_projected.at<cv::Vec3b>(vv, uu)[2] = 0;
            }
          }
        }
        hori_coords.push_back(horiPixelID);
      }
    }
    else
    {
      vertPixelID = imageHeight / 2; // room_center投影到图像中央
      for (int du = -5; du <= 5; ++du)
      {
        for (int dv = -5; dv <= 5; ++dv)
        {
          int uu = std::min(imageWidth - 1, std::max(0, horiPixelID + du));
          int vv = std::min(imageHeight - 1, std::max(0, vertPixelID + dv));
          int idx = vv * imageWidth + uu;
          {
            image_projected.at<cv::Vec3b>(vv, uu)[0] = 0;
            image_projected.at<cv::Vec3b>(vv, uu)[1] = 0;
            image_projected.at<cv::Vec3b>(vv, uu)[2] = 255;
          }
        }
      }
      hori_coord_room_center = horiPixelID;
    }
  }

  // Rotate image to center room_center at middle
  if (!(hori_coord_room_center >= 0 && hori_coord_room_center < imageWidth))
  {
    ROS_ERROR("[project_pcl_to_image] Error: hori_coord_room_center (%d) is out of bounds [0, %d).", hori_coord_room_center, imageWidth);
    return rotated_image; // 返回原图像
  }
  else
  {
    int shift = imageWidth / 2 - hori_coord_room_center;
    shift = (shift + imageWidth) % imageWidth; // wrap to [0, imageWidth)

    if (shift != 0)
    {
      // 1. 平移图像
      cv::Mat right_part = image(cv::Rect(imageWidth - shift, 0, shift, imageHeight)).clone();
      cv::Mat left_part = image(cv::Rect(0, 0, imageWidth - shift, imageHeight)).clone();
      cv::hconcat(right_part, left_part, rotated_image);

      // 2. 平移所有水平投影坐标
      for (int &coord : hori_coords)
      {
        coord = (coord + shift) % imageWidth;
      }
      hori_coord_room_center = (hori_coord_room_center + shift) % imageWidth;
    }
  }
  std::sort(hori_coords.begin(), hori_coords.end());
  int u_start = 0, u_end = imageWidth - 1;
  if (!hori_coords.empty())
  {
    int hori_min = hori_coords.front();
    int hori_max = hori_coords.back();

    u_start = hori_min;
    u_end = hori_max;
    u_start = std::max(u_start - 50, 0);
    u_end = std::min(u_end + 50, imageWidth - 1);

    MY_ASSERT(u_end > u_start);
  }
  else
  {
    ROS_ERROR("[project_pcl_to_image] Error: No valid horizontal coordinates found in the point cloud.");
  }

  if (u_end > u_start)
  {
    cv::Mat cropped = rotated_image(cv::Rect(u_start, 0, u_end - u_start, imageHeight)).clone();
    return cropped;
  }
  else
  {
    ROS_ERROR("[project_pcl_to_image] Error: u_end (%d) is not greater than u_start (%d).", u_end, u_start);
    cv::Mat cropped = rotated_image.clone();
    return cropped; // 返回原图像
  }
}


void SensorCoveragePlanner3D::to_json(json &j, const representation_ns::ObjectNodeRep &obj) const
{
  j = json{
      {"object id", obj.object_id_[0]},
      {"label", obj.label_},
      {"confidence", obj.confidence_},
      {"position", {obj.position_.x, obj.position_.y, obj.position_.z}},
  };
}
void SensorCoveragePlanner3D::to_json(json &j, const representation_ns::ViewPointRep &viewpoint) const
{
  j = json{
      {"viewpoint id", viewpoint.GetId()},
      {"position", {viewpoint.GetPosition().x, viewpoint.GetPosition().y, viewpoint.GetPosition().z}},
      {"room id", viewpoint.room_id_},
  };
}
void SensorCoveragePlanner3D::to_json(json &j, const representation_ns::RoomNodeRep &room) const
{
  std::set<std::string> obj_labels;
  std::set<int> obj_ids;
  // for (const auto &viewpoint_rep_ind : room.viewpoint_indices_)
  // {
  //   const auto &viewpoint_rep = representation_->GetViewPointRepNode(viewpoint_rep_ind);
  //   for (const auto &obj_ind : viewpoint_rep.GetObjectIndices())
  //   {
  //     const auto &object_node = representation_->GetObjectNodeRep(obj_ind);
  //     obj_labels.insert(object_node.label_);
  //     obj_ids.insert(object_node.object_id_[0]);
  //   }
  // }
  for (const auto &obj_ind : room.GetObjectIndices())
  {
    if (!representation_->HasObjectNode(obj_ind))
    {
      ROS_WARN("Object with id %d does not exist in representation, skip", obj_ind);
      continue;
    }
    const auto &object_node = representation_->GetObjectNodeRep(obj_ind);
    obj_labels.insert(object_node.label_);
    obj_ids.insert(object_node.object_id_[0]);
  }
  geometry_msgs::Point goal_point = room.anchor_point_;
  geometry_msgs::Point start_point = robot_position_;
  nav_msgs::Path path;
  double path_distance = keypose_graph_->GetShortestPath(start_point, goal_point, false, path, true);
  std::string label = room.GetRoomLabel();
  j = json{
      {"room id", room.id_},
      {"label", label},
      {"objects", obj_labels},
      {"distance", path_distance}};
}

void SensorCoveragePlanner3D::to_json(json &j, const representation_ns::Representation &rep) const
{
  j = json{
      {"rooms", json::array()},
  };
  // for (const auto &room : rep.room_nodes_)
  for (const auto &id_room_node_pair : rep.GetRoomNodesMap()) 
  {
    auto & room = id_room_node_pair.second;
    json room_json;
    // only if when the room is labeled or has objects
    if ((room.IsLabeled()) && !room.IsCovered() && room.id_ != current_room_id_ && room.is_connected_) {
      to_json(room_json, room);
      j["rooms"].push_back(room_json);
    }
  }
  // print the json as a string
  std::string json_str = j.dump(4); // 4 is the indentation level
  ROS_INFO("Representation JSON:\n%s", json_str.c_str());
}

void SensorCoveragePlanner3D::CheckObjectFound()
{
  if (!initialized_) {
    ROS_ERROR("Planner not initialized, cannot check object found");
    return;
  }

  if (found_object_)
  {
    if (!representation_->HasObjectNode(found_object_id_))
    {
      ROS_WARN("The previously found object with id %d is no longer in the representation, reset found_object_ to false", found_object_id_);
      ResetFoundObjectInfo();
      return;
    }
    // get the room of the found object
    found_object_room_id_ = representation_->GetObjectNodeRep(found_object_id_).room_id_;
    found_object_position_ = representation_->GetObjectNodeRep(found_object_id_).position_;
    if (found_object_room_id_ != -1 && (representation_->HasRoomNode(found_object_room_id_)))
    {
      auto &found_object_room = representation_->GetRoomNode(found_object_room_id_);
      // if the anchor point of the room is not the initial value
      if (found_object_room.anchor_point_.x == 0.0 &&
          found_object_room.anchor_point_.y == 0.0 &&
          found_object_room.anchor_point_.z == 0.0)
      {
        ROS_ERROR("Anchor point of the room %d is not set", found_object_room_id_);
        return;
      }
      geometry_msgs::PointStamped::Ptr geomsg(
          new geometry_msgs::PointStamped());
      geomsg->header.frame_id = "map";
      geomsg->header.stamp = ros::Time::now();
      geomsg->point.x = found_object_room.anchor_point_.x;
      geomsg->point.y = found_object_room.anchor_point_.y;
      geomsg->point.z = found_object_room.anchor_point_.z;
      GoalPointCallback(geomsg);
    }

    ROS_INFO("✅✅✅Target object %s with id %d found", target_object_.c_str(), found_object_id_);
    nav_msgs::Path path;
    found_object_distance_ = keypose_graph_->GetShortestPath(robot_position_, found_object_position_, false, path, true);
    double euclidean_distance_to_object_ = std::sqrt(std::pow(robot_position_.x - found_object_position_.x, 2) +
                                                     std::pow(robot_position_.y - found_object_position_.y, 2) +
                                                     std::pow(robot_position_.z - found_object_position_.z, 2));
    ROS_INFO("Distance to the found object: %.2f meters", found_object_distance_);
    if (found_object_distance_ < 1.0 && euclidean_distance_to_object_ < 2.0)
    {
      ask_found_object_ = true;
    }
    else
    {
      ask_found_object_ = false;
    }

    if (current_room_id_ == found_object_room_id_)
    {
      SetFoundTargetObject();
    }
  }

  std::vector<int> error_object_ids = {};
  // Check if there are any target objects in the current viewpoint representation
  for (auto &id_object_node_pair : representation_->GetObjectNodeRepMapMutable())
  {
    auto &id = id_object_node_pair.first;
    auto &object_node = id_object_node_pair.second;
    if (object_node.label_ == target_object_)
    {
      if (object_node.IsConsidered() || object_node.IsConsideredStrong())
      {
        ROS_INFO("❌❌❌Object %s with id %d already considered, skip", object_node.label_.c_str(), id);
        continue;
      }
      if (kUseVlm && not object_node.is_asked_vlm_)
      {
        ROS_INFO("❌❌❌Object %s with id %d label haven't been checked by VLM, skip", object_node.label_.c_str(), object_node.object_id_[0]);
        continue;
      }

      if (!kUseVlm)
      {
        auto found_object_msg = boost::make_shared<tare_planner::TargetObject>();
        found_object_msg->header.stamp = ros::Time::now();
        found_object_msg->object_id = object_node.object_id_[0];
        found_object_msg->object_label = object_node.label_;
        found_object_msg->img_path = object_node.img_path_;
        found_object_msg->is_target = true;
        TargetObjectCallback(found_object_msg);
        object_node.SetIsConsidered(true);
        considered_object_ids_.insert(id);
        continue;
      }

      int room_id = object_node.room_id_;
      if (!representation_->HasRoomNode(room_id))
      {
        ROS_WARN("❌❌❌Object %s with id %d found in unknown room with id %d",
                    object_node.label_.c_str(), object_node.object_id_[0], room_id);
        continue;
      }
      std::string img_path = object_node.img_path_;
      // if this path does not exist, warn and remove the object and continue
      if (!std::filesystem::exists(img_path))
      {
        ROS_ERROR("❌❌❌Image path %s does not exist, remove the object %s with id %d from consideration",
                    img_path.c_str(), object_node.label_.c_str(), object_node.object_id_[0]);
        error_object_ids.push_back(id);
        continue;
      }
      auto &room_node = representation_->GetRoomNode(room_id);
      std::string label = room_node.GetRoomLabel();
      
      ROS_ERROR("❌❌❌Object %s with id %d in room %s with is_considered_ %d, is_asked_vlm_ %d, visible_viewpoint_indices_ size %d",
              object_node.label_.c_str(), object_node.object_id_[0], label.c_str(), object_node.IsConsidered(), object_node.is_asked_vlm_, (int)object_node.visible_viewpoint_indices_.size());

      if (spatial_condition_=="")
      {
        // publish a targrt object query
        tare_planner::TargetObject target_object_msg;
        target_object_msg.header.stamp = ros::Time::now();
        target_object_msg.object_id = object_node.object_id_[0];
        target_object_msg.object_label = object_node.label_;
        target_object_msg.img_path = object_node.img_path_;
        target_object_msg.room_label = label;
        target_object_msg.is_target = false;
        target_object_pub_.publish(target_object_msg);

        object_node.SetIsConsidered(true);
        considered_object_ids_.insert(id);

        // // TODO: Unit Test
        // auto found_object_msg = std::make_shared<tare_planner::TargetObject>();
        // found_object_msg->header.stamp = ros::Time::now();
        // found_object_msg->object_id = object_node.object_id_[0];
        // found_object_msg->object_label = object_node.label_;
        // found_object_msg->img_path = object_node.img_path_;
        // found_object_msg->room_label = label;
        // found_object_msg->is_target = true;
        // TargetObjectCallback(found_object_msg);
      }
      else
      {
        // get the viewpoint ids which can see this object
        std::vector<int> viewpoint_ids;
        for (const auto &viewpoint_id : object_node.visible_viewpoint_indices_)
        {
          viewpoint_ids.push_back(viewpoint_id);
        }
        // publish a targrt object query
        tare_planner::TargetObjectWithSpatial target_object_msg;
        target_object_msg.header.stamp = ros::Time::now();
        target_object_msg.object_id = object_node.object_id_[0];
        target_object_msg.object_label = object_node.label_;
        target_object_msg.img_path = object_node.img_path_;
        target_object_msg.viewpoint_ids = viewpoint_ids;
        target_object_msg.bbox3d = object_node.bbox3d_;
        target_object_msg.room_label = label;
        target_object_msg.is_target = false;
        target_object_spatial_pub_.publish(target_object_msg);

        object_node.SetIsConsidered(true);
        considered_object_ids_.insert(id);

        // // TODO: Unit Test
        // auto found_object_msg = std::make_shared<tare_planner::TargetObject>();
        // found_object_msg->header.stamp = ros::Time::now();
        // found_object_msg->object_id = object_node.object_id_[0];
        // found_object_msg->object_label = object_node.label_;
        // found_object_msg->img_path = object_node.img_path_;
        // found_object_msg->room_label = label;
        // found_object_msg->is_target = true;
        // TargetObjectCallback(found_object_msg);
      }
    }
  }

  // // remove the error object ids from the representation
  // for (const auto &error_id : error_object_ids)
  // {
  //   representation_->GetObjectNodeRepMapMutable().erase(error_id);
  //   representation_->latest_object_node_rep_map_.erase(error_id);
  //   object_ids_to_remove_.push_back(error_id);
  // }
  
  return;
}

void SensorCoveragePlanner3D::CheckAnchorObjectFound()
{
  if (!initialized_)
  {
    ROS_ERROR("Planner not initialized, cannot check object found");
    return;
  }

  std::vector<int> error_object_ids = {};
  // Check if there are any target objects in the current viewpoint representation
  for (auto &id_object_node_pair : representation_->GetObjectNodeRepMapMutable())
  {
    auto &id = id_object_node_pair.first;
    auto &object_node = id_object_node_pair.second;
    if (object_node.label_ == anchor_object_)
    {
      if (object_node.IsConsidered() || object_node.IsConsideredStrong())
      {
        ROS_INFO("❌❌❌Object %s with id %d already considered, skip", object_node.label_.c_str(), id);
        continue;
      }
      int room_id = object_node.room_id_;
      if (!representation_->HasRoomNode(room_id))
      {
        ROS_WARN("❌❌❌Object %s with id %d found in unknown room with id %d",
                    object_node.label_.c_str(), object_node.object_id_[0], room_id);
        continue;
      }
      if (kUseVlm && not object_node.is_asked_vlm_)
      {
        ROS_INFO("❌❌❌Object %s with id %d label haven't been checked by VLM, skip", object_node.label_.c_str(), object_node.object_id_[0]);
        continue;
      }
      representation_ns::RoomNodeRep &room_node = representation_->GetRoomNode(room_id);
      std::string room_condition_tmp = "in the " + room_node.GetRoomLabel();
      if (room_condition_tmp != room_condition_)
      {
        ROS_INFO("❌❌❌Object %s with id %d in room %s does not satisfy the room condition %s, skip",
                    object_node.label_.c_str(), object_node.object_id_[0], room_node.GetRoomLabel().c_str(), room_condition_.c_str());
        continue;
      }
      std::string img_path = object_node.img_path_;
      // if this path does not exist, warn and remove the object and continue
      if (!std::filesystem::exists(img_path))
      {
        ROS_ERROR("❌❌❌Image path %s does not exist, remove the object %s with id %d from consideration",
                     img_path.c_str(), object_node.label_.c_str(), object_node.object_id_[0]);
        error_object_ids.push_back(id);
        continue;
      }
      std::string label = room_node.GetRoomLabel();

      ROS_ERROR("❌❌❌Object %s with id %d in room %s with is_considered_ %d, is_asked_vlm_ %d, visible_viewpoint_indices_ size %d",
                   object_node.label_.c_str(), object_node.object_id_[0], label.c_str(), object_node.IsConsidered(), object_node.is_asked_vlm_, (int)object_node.visible_viewpoint_indices_.size());

      // publish a targrt object query
      tare_planner::TargetObject anchor_object_msg;
      anchor_object_msg.header.stamp = ros::Time::now();
      anchor_object_msg.object_id = object_node.object_id_[0];
      anchor_object_msg.object_label = object_node.label_;
      anchor_object_msg.img_path = object_node.img_path_;
      anchor_object_msg.room_label = label;
      anchor_object_msg.is_target = false;
      anchor_object_pub_.publish(anchor_object_msg);

      object_node.SetIsConsidered(true);
      considered_object_ids_.insert(id);
    }
  }

  if (found_anchor_object_ && !found_object_)
  {
    if (!representation_->HasObjectNode(found_anchor_object_id_))
    {
      ROS_WARN("The previously found anchor object with id %d is no longer in the representation, reset found_anchor_object_ to false", found_anchor_object_id_);
      ResetFoundAnchorObjectInfo();
      return;
    }
    // get the room of the found object
    auto &object_node = representation_->GetObjectNodeRep(found_anchor_object_id_);
    found_anchor_object_room_id_ = object_node.room_id_;
    found_anchor_object_position_ = object_node.position_;

    found_anchor_object_viewpoint_positions_.clear();
    for (const auto &viewpoint_id : object_node.visible_viewpoint_indices_)
    {
      // check if the viewpoint is very close to the robot position, if yes, consider it as visited
      auto &viewpoint = representation_->GetViewPointRepNode(viewpoint_id);
      auto &vp_pos = viewpoint.GetPosition();
      double distance_to_robot = std::sqrt(std::pow(robot_position_.x - vp_pos.x, 2) +
                                           std::pow(robot_position_.y - vp_pos.y, 2));
      if (distance_to_robot < 1.0)
      {
        found_anchor_object_viewpoint_positions_visited_.push_back(viewpoint.GetPosition());
        ROS_INFO("Viewpoint %d is very close to the robot position, consider it as visited", viewpoint.GetId());
        continue;
      }
      // only add the viewpoint position if it is in the same room as the anchor object,
      // and it has not been considered before
      // and it is not already in the list
      bool already_considered = std::find(found_anchor_object_viewpoint_positions_visited_.begin(), found_anchor_object_viewpoint_positions_visited_.end(), viewpoint.GetPosition()) != found_anchor_object_viewpoint_positions_visited_.end();
      if (viewpoint.room_id_ == found_anchor_object_room_id_ && !already_considered)
      {
        found_anchor_object_viewpoint_positions_.push_back(viewpoint.GetPosition());
      }
    }

    if (found_anchor_object_room_id_ != -1 && (representation_->HasRoomNode(found_anchor_object_room_id_)))
    {
      auto &found_anchor_object_room = representation_->GetRoomNode(found_anchor_object_room_id_);
      // if the anchor point of the room is not the initial value
      if (found_anchor_object_room.anchor_point_.x == 0.0 &&
          found_anchor_object_room.anchor_point_.y == 0.0 &&
          found_anchor_object_room.anchor_point_.z == 0.0)
      {
        ROS_ERROR("Anchor point of the room %d is not set", found_anchor_object_room_id_);
        return;
      }
      geometry_msgs::PointStamped::Ptr geomsg(
          new geometry_msgs::PointStamped());
      geomsg->header.frame_id = "map";
      geomsg->header.stamp = ros::Time::now();
      geomsg->point.x = found_anchor_object_room.anchor_point_.x;
      geomsg->point.y = found_anchor_object_room.anchor_point_.y;
      geomsg->point.z = found_anchor_object_room.anchor_point_.z;
      GoalPointCallback(geomsg);
    }

    ROS_INFO("✅✅✅Anchor object %s with id %d found", anchor_object_.c_str(), found_anchor_object_id_);
    nav_msgs::Path path;
    found_anchor_object_distance_ = keypose_graph_->GetShortestPath(robot_position_, found_anchor_object_position_, false, path, true);
    double euclidean_distance_to_object_ = std::sqrt(std::pow(robot_position_.x - found_anchor_object_position_.x, 2) +
                                                     std::pow(robot_position_.y - found_anchor_object_position_.y, 2) +
                                                     std::pow(robot_position_.z - found_anchor_object_position_.z, 2));
    ROS_INFO("Distance to the found object: %.2f meters", found_anchor_object_distance_);
    // if (found_object_distance_ < 1.0 && euclidean_distance_to_object_ < 2.0)
    // {
    //   ask_found_object_ = true;
    // }
    // else
    // {
    //   ask_found_object_ = false;
    // }

    if (current_room_id_ == found_anchor_object_room_id_)
    {
      SetFoundAnchorObject();
    }
  }
  return;
}


bool SensorCoveragePlanner3D::SelectNearestUnexploredRoom()
{
  int best_room_id = -1;
  double best_dist = std::numeric_limits<double>::max();

  for (const auto& id_room_pair : representation_->GetRoomNodesMap()) {
    const auto& room = id_room_pair.second;
    if (room.id_ == current_room_id_) {
      continue;
    }
    if (room.IsCovered()) {
      continue;
    }
    if (!room.is_connected_) {
      continue;
    }
    double dist = misc_utils_ns::PointXYDist(robot_position_, room.anchor_point_);
    if (dist < best_dist) {
      best_dist = dist;
      best_room_id = room.id_;
    }
  }

  if (best_room_id < 0) {
    ROS_INFO("No unexplored room found, exploration may be complete");
    has_candidate_room_position_ = false;
    candidate_room_id_ = -1;
    return false;
  }

  const auto& room_node = representation_->GetRoomNode(best_room_id);
  candidate_room_position_ = room_node.anchor_point_;
  candidate_room_id_ = best_room_id;
  has_candidate_room_position_ = true;
  ROS_INFO("Selected nearest unexplored room %d at distance %.2f", best_room_id, best_dist);
  return true;
}

void SensorCoveragePlanner3D::GetAnswer()
{
  if (!initialized_) {
    ROS_ERROR("Planner not initialized, cannot get answer");
    return;
  }
  if (!has_candidate_room_position_) {
    if (!SelectNearestUnexploredRoom()) {
      return;
    }
  }

  geometry_msgs::PointStamped::Ptr geomsg(new geometry_msgs::PointStamped());
  geomsg->header.frame_id = "map";
  geomsg->header.stamp = ros::Time::now();
  geomsg->point = candidate_room_position_;
  has_candidate_room_position_ = false;
  asked_in_advance_ = false;
  GoalPointCallback(geomsg);
  SetStartAndEndRoomId();
}

void SensorCoveragePlanner3D::PublishObjectNodeMarkers()
{
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker clear_marker;
  clear_marker.header.frame_id = kWorldFrameID;
  clear_marker.header.stamp = ros::Time::now();
  clear_marker.ns = "object_nodes_room";
  clear_marker.id = 0;
  clear_marker.type = visualization_msgs::Marker::DELETEALL;
  marker_array.markers.push_back(clear_marker);
  for (const auto &id_object_node_pair : representation_->GetObjectNodeRepMap())
  {
    const auto &object_node = id_object_node_pair.second;
    visualization_msgs::Marker marker;
    marker.header.frame_id = kWorldFrameID;
    marker.header.stamp = ros::Time::now();
    marker.ns = "object_nodes_room";
    marker.id = object_node.object_id_[0];
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position = object_node.position_;
    marker.pose.position.z += 0.5; // raise the text a bit
    marker.pose.orientation.w = 1.0;
    marker.scale.z = 0.2;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.5;
    marker.color.b = 0.0;
    marker.text = "O" + std::to_string(object_node.object_id_[0]) + " R" + std::to_string(object_node.room_id_);
    marker_array.markers.push_back(marker);
  }
  object_node_marker_pub_.publish(marker_array);
}

void SensorCoveragePlanner3D::ProcessObjectNodes()
{
  for (auto &obj_id : object_ids_to_remove_)
  {
    representation_->GetObjectNodeRepMapMutable().erase(obj_id);
    representation_->GetLatestObjectNodeIndicesMutable().erase(obj_id);

    for (auto &viewpoint : representation_->GetViewPointRepsMutable())
    {
      viewpoint.DeleteObjectIndex(obj_id);
      viewpoint.DeleteDirectObjectIndex(obj_id);
    }
    for (auto &id_room_pair : representation_->GetRoomNodesMapMutable())
    {
      id_room_pair.second.DeleteObjectIndex(obj_id);
    }
  }
  // if the found_object_id_ or the found_anchor_object_id_ is in the removed list, reset the found info or the anchor found info
  if (found_object_ && (std::find(object_ids_to_remove_.begin(), object_ids_to_remove_.end(), found_object_id_) != object_ids_to_remove_.end()))
  {
    ROS_WARN("The previously found object with id %d has been removed from the representation, reset found_object_ to false", found_object_id_);
    ResetFoundObjectInfo();
  }
  if (found_anchor_object_ && (std::find(object_ids_to_remove_.begin(), object_ids_to_remove_.end(), found_anchor_object_id_) != object_ids_to_remove_.end()))
  {
    ROS_WARN("The previously found anchor object with id %d has been removed from the representation, reset found_anchor_object_ to false", found_anchor_object_id_);
    ResetFoundAnchorObjectInfo();
  }
  object_ids_to_remove_.clear();
}

void SensorCoveragePlanner3D::executeTimer(const ros::TimerEvent& /*event*/) {
  execute();
}

} // namespace sensor_coverage_planner_3d_ns
