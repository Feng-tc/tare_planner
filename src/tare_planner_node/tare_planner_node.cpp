#include <ros/ros.h>
#include "sensor_coverage_planner/sensor_coverage_planner_ground.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tare_planner_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  sensor_coverage_planner_3d_ns::SensorCoveragePlanner3D planner(nh, private_nh);
  planner.initialize();
  ros::spin();
  return 0;
}
