#!/usr/bin/env python3
"""Phase 2 ROS2 -> ROS1 fixes for tare_planner."""
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DIRS = [ROOT / "include", ROOT / "src"]


def port_content(text: str) -> str:
    # logging
    text = re.sub(r'RCLCPP_WARN_STREAM\(rclcpp::get_logger\("[^"]+"\),\s*', "ROS_WARN_STREAM(", text)
    text = re.sub(r'RCLCPP_ERROR_STREAM\(rclcpp::get_logger\("[^"]+"\),\s*', "ROS_ERROR_STREAM(", text)
    text = re.sub(r'RCLCPP_ERROR\(rclcpp::get_logger\("[^"]+"\),\s*', "ROS_ERROR(", text)
    text = re.sub(r'RCLCPP_WARN\(rclcpp::get_logger\("[^"]+"\),\s*', "ROS_WARN(", text)
    text = re.sub(r'RCLCPP_INFO\(rclcpp::get_logger\("[^"]+"\),\s*', "ROS_INFO(", text)
    text = re.sub(
        r'RCLCPP_WARN\(nh_->get_logger\(\),\s*',
        "ROS_WARN(",
        text,
    )
    text = re.sub(
        r'RCLCPP_ERROR\(nh_->get_logger\(\),\s*',
        "ROS_ERROR(",
        text,
    )

    # get_parameter("k", v) -> param with same default placeholder (caller must set defaults in yaml)
    text = re.sub(
        r'(?:this->|nh->|private_nh_\.|nh_\.|pnh\.|nh\.)get_parameter\("([^"]+)",\s*([^)]+)\)',
        r'nh.param("\1", \2, \2)',
        text,
    )
    text = re.sub(
        r'this->get_parameter\("([^"]+)"\)\.as_double\(\)',
        r'([&](){ static double _v=0.0; nh_.param("\1", _v, _v); return _v; }())',
        text,
    )
    text = re.sub(
        r'this->get_parameter\("([^"]+)"\)\.as_int\(\)',
        r'([&](){ static int _v=0; nh_.param("\1", _v, _v); return _v; }())',
        text,
    )
    text = re.sub(
        r'nh->get_parameter\("([^"]+)"\)\.as_double\(\)',
        r'([&](){ double _v=0.0; nh->param("\1", _v, _v); return _v; }())',
        text,
    )
    text = re.sub(
        r'nh->get_parameter\("([^"]+)"\)\.as_int\(\)',
        r'([&](){ int _v=0; nh->param("\1", _v, _v); return _v; }())',
        text,
    )

    text = text.replace("rclcpp::QoS(1).transient_local()", "1")
    text = text.replace("rclcpp::ok()", "ros::ok()")
    text = text.replace("rclcpp::Rate", "ros::Rate")
    text = text.replace("rclcpp::spin_some", "ros::spinOnce")
    text = text.replace("rclcpp::init", "ros::init")
    text = text.replace("rclcpp::spin", "ros::spin")
    text = text.replace("rclcpp::shutdown", "ros::shutdown")
    text = text.replace("rclcpp::Node::make_shared", "std::make_shared<ros::NodeHandle>")

    text = text.replace("shared_from_this()", "&nh_")
    text = text.replace("this->create_publisher<", "nh_.advertise<")
    text = text.replace("this->create_subscription<", "nh_.subscribe<")
    text = text.replace("nh->create_publisher<", "nh->advertise<")
    text = text.replace("nh->create_subscription<", "nh->subscribe<")
    text = text.replace("->create_publisher<", ".advertise<")
    text = text.replace("->create_subscription<", ".subscribe<")

    return text


def main():
    for base in DIRS:
        for path in base.rglob("*"):
            if path.suffix not in (".cpp", ".h"):
                continue
            original = path.read_text(encoding="utf-8")
            ported = port_content(original)
            if ported != original:
                path.write_text(ported, encoding="utf-8")
                print(path.relative_to(ROOT))


if __name__ == "__main__":
    main()
