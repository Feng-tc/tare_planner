/**
 * @file tare_visualizer.cpp
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that visualizes the planning process
 * @version 0.1
 * @date 2021-06-01
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "tare_visualizer/tare_visualizer.h"

namespace tare_visualizer_ns
{
TAREVisualizer::TAREVisualizer(ros::NodeHandle* nh)
{
  ReadParameters(nh);

  marker_publisher_ = nh->advertise<visualization_msgs::Marker>("tare_visualizer/marker", 1);
  local_path_publisher_ = nh->advertise<nav_msgs::Path>("tare_visualizer/local_path", 1);

  global_subspaces_marker_ =
      std::make_shared<misc_utils_ns::Marker>(nh, "tare_visualizer/exploring_subspaces", kWorldFrameID);
  local_planning_horizon_marker_ =
      std::make_shared<misc_utils_ns::Marker>(nh, "tare_visualizer/local_planning_horizon", kWorldFrameID);

  uncovered_surface_point_cloud_ = std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
      nh, "tare_visualizer/uncovered_surface_points", kWorldFrameID);
  viewpoint_candidate_cloud_ = std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
      nh, "tare_visualizer/viewpoint_candidates", kWorldFrameID);
  viewpoint_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "tare_visualizer/viewpoints", kWorldFrameID);

  InitializeMarkers();
}
bool TAREVisualizer::ReadParameters(ros::NodeHandle* nh)
{
  nh->param("kExploringSubspaceMarkerColorGradientAlpha", kExploringSubspaceMarkerColorGradientAlpha, kExploringSubspaceMarkerColorGradientAlpha);
  nh->param("kExploringSubspaceMarkerColorMaxAlpha", kExploringSubspaceMarkerColorMaxAlpha, kExploringSubspaceMarkerColorMaxAlpha);
  kExploringSubspaceMarkerColor.r = ([&](){ double _v=0.0; nh->param("kExploringSubspaceMarkerColorR", _v, _v); return _v; }());
  kExploringSubspaceMarkerColor.g = ([&](){ double _v=0.0; nh->param("kExploringSubspaceMarkerColorG", _v, _v); return _v; }());
  kExploringSubspaceMarkerColor.b = ([&](){ double _v=0.0; nh->param("kExploringSubspaceMarkerColorB", _v, _v); return _v; }());
  kExploringSubspaceMarkerColor.a = ([&](){ double _v=0.0; nh->param("kExploringSubspaceMarkerColorA", _v, _v); return _v; }());
  kLocalPlanningHorizonMarkerColor.r = ([&](){ double _v=0.0; nh->param("kLocalPlanningHorizonMarkerColorR", _v, _v); return _v; }());
  kLocalPlanningHorizonMarkerColor.g = ([&](){ double _v=0.0; nh->param("kLocalPlanningHorizonMarkerColorG", _v, _v); return _v; }());
  kLocalPlanningHorizonMarkerColor.b = ([&](){ double _v=0.0; nh->param("kLocalPlanningHorizonMarkerColorB", _v, _v); return _v; }());
  kLocalPlanningHorizonMarkerColor.a = ([&](){ double _v=0.0; nh->param("kLocalPlanningHorizonMarkerColorA", _v, _v); return _v; }());
  nh->param("kLocalPlanningHorizonMarkerColorA", kLocalPlanningHorizonMarkerWidth, kLocalPlanningHorizonMarkerWidth);
  int viewpoint_num_x = ([&](){ int _v=0; nh->param("viewpoint_manager/number_x", _v, _v); return _v; }());
  int viewpoint_num_y = ([&](){ int _v=0; nh->param("viewpoint_manager/number_y", _v, _v); return _v; }());
  double viewpoint_resolution_x = ([&](){ double _v=0.0; nh->param("viewpoint_manager/resolution_x", _v, _v); return _v; }());
  double viewpoint_resolution_y = ([&](){ double _v=0.0; nh->param("viewpoint_manager/resolution_x", _v, _v); return _v; }());
  kGlobalSubspaceSize = viewpoint_num_x * viewpoint_resolution_x / 5;
  kLocalPlanningHorizonSizeX = viewpoint_num_x * viewpoint_resolution_x;
  kLocalPlanningHorizonSizeY = viewpoint_num_y * viewpoint_resolution_y;
  nh->param("kGridWorldCellHeight", kGlobalSubspaceHeight, kGlobalSubspaceHeight);
  nh->param("kLocalPlanningHorizonHeight", kLocalPlanningHorizonSizeZ, kLocalPlanningHorizonSizeZ);

  return true;
}

void TAREVisualizer::InitializeMarkers()
{
  global_subspaces_marker_->SetType(visualization_msgs::Marker::CUBE_LIST);
  global_subspaces_marker_->SetScale(kGlobalSubspaceSize, kGlobalSubspaceSize, kGlobalSubspaceHeight);
  global_subspaces_marker_->SetColorRGBA(kExploringSubspaceMarkerColor);

  local_planning_horizon_marker_->SetType(visualization_msgs::Marker::LINE_LIST);
  local_planning_horizon_marker_->SetScale(kLocalPlanningHorizonMarkerWidth, 0, 0);
  local_planning_horizon_marker_->SetColorRGBA(kExploringSubspaceMarkerColor);
}

void TAREVisualizer::GetLocalPlanningHorizonMarker(double x, double y, double z)
{
  local_planning_horizon_origin_.x = x;
  local_planning_horizon_origin_.y = y;
  local_planning_horizon_origin_.z = z - kLocalPlanningHorizonSizeZ / 2;

  geometry_msgs::Point upper_right;
  upper_right.x = local_planning_horizon_origin_.x + kLocalPlanningHorizonSizeX;
  upper_right.y = local_planning_horizon_origin_.y + kLocalPlanningHorizonSizeY;
  upper_right.z = local_planning_horizon_origin_.z + kLocalPlanningHorizonSizeZ;

  geometry_msgs::Point lower_right;
  lower_right.x = local_planning_horizon_origin_.x;
  lower_right.y = local_planning_horizon_origin_.y + kLocalPlanningHorizonSizeY;
  lower_right.z = local_planning_horizon_origin_.z + kLocalPlanningHorizonSizeZ;

  geometry_msgs::Point upper_left;
  upper_left.x = local_planning_horizon_origin_.x + kLocalPlanningHorizonSizeX;
  upper_left.y = local_planning_horizon_origin_.y;
  upper_left.z = local_planning_horizon_origin_.z + kLocalPlanningHorizonSizeZ;

  geometry_msgs::Point lower_left;
  lower_left.x = local_planning_horizon_origin_.x;
  lower_left.y = local_planning_horizon_origin_.y;
  lower_left.z = local_planning_horizon_origin_.z + kLocalPlanningHorizonSizeZ;

  geometry_msgs::Point upper_right2;
  upper_right2.x = local_planning_horizon_origin_.x + kLocalPlanningHorizonSizeX;
  upper_right2.y = local_planning_horizon_origin_.y + kLocalPlanningHorizonSizeY;
  upper_right2.z = local_planning_horizon_origin_.z;

  geometry_msgs::Point lower_right2;
  lower_right2.x = local_planning_horizon_origin_.x;
  lower_right2.y = local_planning_horizon_origin_.y + kLocalPlanningHorizonSizeY;
  lower_right2.z = local_planning_horizon_origin_.z;

  geometry_msgs::Point upper_left2;
  upper_left2.x = local_planning_horizon_origin_.x + kLocalPlanningHorizonSizeX;
  upper_left2.y = local_planning_horizon_origin_.y;
  upper_left2.z = local_planning_horizon_origin_.z;

  geometry_msgs::Point lower_left2;
  lower_left2.x = local_planning_horizon_origin_.x;
  lower_left2.y = local_planning_horizon_origin_.y;
  lower_left2.z = local_planning_horizon_origin_.z;

  local_planning_horizon_marker_->marker_.points.clear();

  local_planning_horizon_marker_->marker_.points.push_back(upper_right);
  local_planning_horizon_marker_->marker_.points.push_back(lower_right);
  local_planning_horizon_marker_->marker_.points.push_back(lower_right);
  local_planning_horizon_marker_->marker_.points.push_back(lower_left);
  local_planning_horizon_marker_->marker_.points.push_back(lower_left);
  local_planning_horizon_marker_->marker_.points.push_back(upper_left);
  local_planning_horizon_marker_->marker_.points.push_back(upper_left);
  local_planning_horizon_marker_->marker_.points.push_back(upper_right);

  local_planning_horizon_marker_->marker_.points.push_back(upper_right);
  local_planning_horizon_marker_->marker_.points.push_back(upper_right2);
  local_planning_horizon_marker_->marker_.points.push_back(upper_left);
  local_planning_horizon_marker_->marker_.points.push_back(upper_left2);
  local_planning_horizon_marker_->marker_.points.push_back(lower_left);
  local_planning_horizon_marker_->marker_.points.push_back(lower_left2);
  local_planning_horizon_marker_->marker_.points.push_back(lower_right);
  local_planning_horizon_marker_->marker_.points.push_back(lower_right2);

  local_planning_horizon_marker_->marker_.points.push_back(upper_right2);
  local_planning_horizon_marker_->marker_.points.push_back(lower_right2);
  local_planning_horizon_marker_->marker_.points.push_back(lower_right2);
  local_planning_horizon_marker_->marker_.points.push_back(lower_left2);
  local_planning_horizon_marker_->marker_.points.push_back(lower_left2);
  local_planning_horizon_marker_->marker_.points.push_back(upper_left2);
  local_planning_horizon_marker_->marker_.points.push_back(upper_left2);
  local_planning_horizon_marker_->marker_.points.push_back(upper_right2);
}

void TAREVisualizer::GetGlobalSubspaceMarker(const std::shared_ptr<grid_world_ns::GridWorld>& grid_world,
                                             const std::vector<int>& ordered_cell_indices)
{
  global_subspaces_marker_->marker_.points.clear();
  global_subspaces_marker_->marker_.colors.clear();
  int cell_num = ordered_cell_indices.size();
  for (int i = 0; i < cell_num; i++)
  {
    int cell_ind = ordered_cell_indices[i];
    if (!grid_world->IndInBound(cell_ind))
    {
      continue;
    }
    geometry_msgs::Point cell_center = grid_world->GetCellPosition(cell_ind);
    std_msgs::ColorRGBA color;
    color.r = 0.0;
    color.g = 1.0;
    color.b = 0.0;
    if (kExploringSubspaceMarkerColorGradientAlpha)
    {
      color.a = ((cell_num - i) * 1.0 / cell_num) * kExploringSubspaceMarkerColorMaxAlpha;
    }
    else
    {
      color.a = 1.0;
    }
    global_subspaces_marker_->marker_.points.push_back(cell_center);
    global_subspaces_marker_->marker_.colors.push_back(color);
  }
}

void TAREVisualizer::PublishMarkers()
{
  local_planning_horizon_marker_->Publish();
  if (!global_subspaces_marker_->marker_.points.empty())
  {
    global_subspaces_marker_->SetAction(visualization_msgs::Marker::ADD);
    global_subspaces_marker_->Publish();
  }
  else
  {
    global_subspaces_marker_->SetAction(visualization_msgs::Marker::DELETE);
    global_subspaces_marker_->Publish();
  }
}

}  // namespace tare_visualizer_ns
