#!/usr/bin/env python3
"""Mechanical ROS2 -> ROS1 conversions for tare_planner source files."""
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DIRS = [ROOT / "include", ROOT / "src"]

INCLUDE_MAP = {
    "geometry_msgs/msg/point_stamped.hpp": "geometry_msgs/PointStamped.h",
    "geometry_msgs/msg/polygon_stamped.hpp": "geometry_msgs/PolygonStamped.h",
    "geometry_msgs/msg/pose.hpp": "geometry_msgs/Pose.h",
    "geometry_msgs/msg/pose_stamped.hpp": "geometry_msgs/PoseStamped.h",
    "geometry_msgs/msg/point.hpp": "geometry_msgs/Point.h",
    "geometry_msgs/msg/polygon.hpp": "geometry_msgs/Polygon.h",
    "geometry_msgs/msg/quaternion.hpp": "geometry_msgs/Quaternion.h",
    "nav_msgs/msg/odometry.hpp": "nav_msgs/Odometry.h",
    "nav_msgs/msg/path.hpp": "nav_msgs/Path.h",
    "sensor_msgs/msg/point_cloud2.hpp": "sensor_msgs/PointCloud2.h",
    "sensor_msgs/msg/image.hpp": "sensor_msgs/Image.h",
    "sensor_msgs/msg/joy.hpp": "sensor_msgs/Joy.h",
    "std_msgs/msg/bool.hpp": "std_msgs/Bool.h",
    "std_msgs/msg/empty.hpp": "std_msgs/Empty.h",
    "std_msgs/msg/float32.hpp": "std_msgs/Float32.h",
    "std_msgs/msg/int32.hpp": "std_msgs/Int32.h",
    "std_msgs/msg/string.hpp": "std_msgs/String.h",
    "std_msgs/msg/int32_multi_array.hpp": "std_msgs/Int32MultiArray.h",
    "std_msgs/msg/color_rgba.hpp": "std_msgs/ColorRGBA.h",
    "std_msgs/msg/header.hpp": "std_msgs/Header.h",
    "visualization_msgs/msg/marker.hpp": "visualization_msgs/Marker.h",
    "visualization_msgs/msg/marker_array.hpp": "visualization_msgs/MarkerArray.h",
    "tare_planner/msg/object_node.hpp": "tare_planner/ObjectNode.h",
    "tare_planner/msg/object_node_list.hpp": "tare_planner/ObjectNodeList.h",
    "tare_planner/msg/room_node.hpp": "tare_planner/RoomNode.h",
    "tare_planner/msg/room_node_list.hpp": "tare_planner/RoomNodeList.h",
    "tare_planner/msg/room_type.hpp": "tare_planner/RoomType.h",
    "tare_planner/msg/viewpoint_rep.hpp": "tare_planner/ViewpointRep.h",
    "tare_planner/msg/detection_result.hpp": "tare_planner/DetectionResult.h",
    "tare_planner/msg/room_early_stop1.hpp": "tare_planner/RoomEarlyStop1.h",
    "tare_planner/msg/vlm_answer.hpp": "tare_planner/VlmAnswer.h",
    "tare_planner/msg/object_type.hpp": "tare_planner/ObjectType.h",
    "tare_planner/msg/navigation_query.hpp": "tare_planner/NavigationQuery.h",
    "tare_planner/msg/target_object_instruction.hpp": "tare_planner/TargetObjectInstruction.h",
    "tare_planner/msg/target_object.hpp": "tare_planner/TargetObject.h",
    "tare_planner/msg/target_object_with_spatial.hpp": "tare_planner/TargetObjectWithSpatial.h",
}


def port_content(text: str) -> str:
    text = text.replace('#include "rclcpp/rclcpp.hpp"', '#include <ros/ros.h>')
    text = text.replace('#include <rclcpp/rclcpp.hpp>', '#include <ros/ros.h>')
    for old, new in INCLUDE_MAP.items():
        text = text.replace(f'#include "{old}"', f'#include <{new}>')
        text = text.replace(f'#include <{old}>', f'#include <{new}>')

    for pkg in ("std_msgs", "sensor_msgs", "geometry_msgs", "nav_msgs", "visualization_msgs", "tare_planner"):
        text = text.replace(f"{pkg}::msg::", f"{pkg}::")

    text = re.sub(r"rclcpp::Time", "ros::Time", text)
    text = re.sub(r"::ConstSharedPtr", "::ConstPtr", text)
    text = re.sub(r"rclcpp::Publisher<([^>]+)>::SharedPtr", r"ros::Publisher", text)
    text = re.sub(r"rclcpp::Subscription<([^>]+)>::SharedPtr", r"ros::Subscriber", text)
    text = re.sub(r"rclcpp::Node::SharedPtr", "ros::NodeHandle*", text)
    text = re.sub(r"rclcpp::TimerBase::SharedPtr", "ros::Timer", text)
    text = re.sub(r"rclcpp::WallTimer::SharedPtr", "ros::Timer", text)

    text = re.sub(
        r"RCLCPP_INFO\s*\(\s*this->get_logger\(\)\s*,\s*",
        "ROS_INFO(",
        text,
    )
    text = re.sub(
        r"RCLCPP_WARN\s*\(\s*this->get_logger\(\)\s*,\s*",
        "ROS_WARN(",
        text,
    )
    text = re.sub(
        r"RCLCPP_ERROR\s*\(\s*this->get_logger\(\)\s*,\s*",
        "ROS_ERROR(",
        text,
    )
    text = re.sub(
        r"RCLCPP_DEBUG\s*\(\s*this->get_logger\(\)\s*,\s*",
        "ROS_DEBUG(",
        text,
    )
    text = re.sub(r"RCLCPP_INFO\s*\(\s*get_logger\(\)\s*,\s*", "ROS_INFO(", text)
    text = re.sub(r"RCLCPP_WARN\s*\(\s*get_logger\(\)\s*,\s*", "ROS_WARN(", text)
    text = re.sub(r"RCLCPP_ERROR\s*\(\s*get_logger\(\)\s*,\s*", "ROS_ERROR(", text)

    text = text.replace("this->now()", "ros::Time::now()")
    text = text.replace("->publish(", ".publish(")
    text = text.replace("using namespace std::chrono_literals;", "")

    return text


def main():
    count = 0
    for base in DIRS:
        for path in base.rglob("*"):
            if path.suffix not in (".cpp", ".h", ".hpp"):
                continue
            original = path.read_text(encoding="utf-8")
            ported = port_content(original)
            if ported != original:
                path.write_text(ported, encoding="utf-8")
                count += 1
                print(path.relative_to(ROOT))
    print(f"Ported {count} files")


if __name__ == "__main__":
    main()
