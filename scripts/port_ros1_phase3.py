#!/usr/bin/env python3
"""Phase 3: declare_parameter cleanup, subscribe bind fix, Node class fix."""
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


def convert_declare_get_params(text: str, nh_prefix: str = "private_nh_") -> str:
    declares = {}
    for m in re.finditer(
        r"this->declare_parameter<([^>]+)>\(\s*\"([^\"]+)\"\s*(?:,\s*([^)]+))?\s*\)\s*;",
        text,
        re.MULTILINE,
    ):
        typ, key, default = m.group(1), m.group(2), m.group(3)
        if default is not None:
            declares[key] = default.strip()

    text = re.sub(
        r"\s*this->declare_parameter<[^>]+>\([^;]+;\n",
        "\n",
        text,
    )
    text = re.sub(
        r"\s*this->get_parameter\(\"([^\"]+)\",\s*([^)]+)\)\s*;",
        lambda m: f'\n  {nh_prefix}.param("{m.group(1)}", {m.group(2)}, {declares.get(m.group(1), m.group(2))});',
        text,
    )
    text = re.sub(
        r"bool got_parameter = true;\s*got_parameter &= this->get_parameter\([^;]+;\s*if \(!got_parameter\) \{[^}]+\}\s*",
        "",
        text,
        flags=re.DOTALL,
    )
    return text


def fix_subscribe_bind(text: str) -> str:
    text = re.sub(
        r"nh_\.subscribe<([^>]+)>\(\s*([^,]+),\s*(\d+),\s*"
        r"std::bind\(&([^:]+)::([^,]+),\s*this,\s*std::placeholders::_1\)\s*\)",
        r"nh_.subscribe<\1>(\2, \3, &\4::\5, this)",
        text,
    )
    text = re.sub(
        r"nh_\.subscribe<([^>]+)>\(\s*([^,]+),\s*(\d+),\s*"
        r"std::bind\(&([^:]+)::([^,]+),\s*this,\s*std::placeholders::_1\),\s*"
        r"[^)]+\)",
        r"nh_.subscribe<\1>(\2, \3, &\4::\5, this)",
        text,
    )
    return text


def fix_timer(text: str) -> str:
    text = re.sub(
        r"execution_timer_ = this->create_wall_timer\(\s*1000ms,\s*"
        r"std::bind\(&SensorCoveragePlanner3D::execute, this\)\s*\)\s*;",
        "execution_timer_ = nh_.createTimer(ros::Duration(1.0), "
        "&SensorCoveragePlanner3D::executeTimer, this);",
        text,
    )
    text = re.sub(
        r"timer_ = this->create_wall_timer\(\s*([^,]+),\s*"
        r"std::bind\(&RoomSegmentationNode::timerCallback, this\)\s*\)\s*;",
        r"timer_ = nh_.createTimer(\1, &RoomSegmentationNode::timerCallback, this);",
        text,
    )
    return text


def port_file(path: Path):
    text = path.read_text(encoding="utf-8")
    original = text
    if "declare_parameter" in text:
        nh_prefix = "private_nh_" if "SensorCoveragePlanner3D" in text or "private_nh_" in text else "pnh_"
        if "RoomSegmentationNode" in text:
            nh_prefix = "pnh_"
        text = convert_declare_get_params(text, nh_prefix)
    text = fix_subscribe_bind(text)
    text = fix_timer(text)
    text = text.replace(": public rclcpp::Node", "")
    text = text.replace("Node(\"tare_planner_node\")", "")
    text = text.replace("Node(\"room_segmentation_node\")", "")
    if text != original:
        path.write_text(text, encoding="utf-8")
        print(path.relative_to(ROOT))


def main():
    targets = [
        ROOT / "src/sensor_coverage_planner/sensor_coverage_planner_ground.cpp",
        ROOT / "include/sensor_coverage_planner/sensor_coverage_planner_ground.h",
        ROOT / "src/room_segmentation/room_segmentation.cpp",
        ROOT / "include/room_segmentation/room_segmentation_node.h",
        ROOT / "src/navigation_boundary_publisher/navigationBoundary.cpp",
    ]
    for path in targets:
        if path.exists():
            port_file(path)


if __name__ == "__main__":
    main()
