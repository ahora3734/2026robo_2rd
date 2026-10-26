#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, LaserScan
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32, String


class SensorMonitor(Node):
    """ROS 2 bring-up monitor. No hardware actuation."""

    def __init__(self):
        super().__init__("sensor_monitor")

        self.names = [
            "/scan",
            "/scan_filtered",
            "/map",
            "/imu_status",
            "/imu_heading",
            "/gps_pos",
            "/nav_path_data",
            "/vision_info",
            "/yolo_stream",
            "/vehicle_cmd",
            "/vehicle_status",
            "/set_destination",
        ]

        self.last_seen = {}
        self.count = {}
        self.last_value = {}

        self.map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(
            LaserScan,
            "/scan",
            lambda m: self.touch("/scan", f"{len(m.ranges)} beams"),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan,
            "/scan_filtered",
            lambda m: self.touch("/scan_filtered", f"{len(m.ranges)} beams"),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            OccupancyGrid,
            "/map",
            lambda m: self.touch(
                "/map", f"{m.info.width}x{m.info.height} res={m.info.resolution:.3f}"
            ),
            self.map_qos,
        )
        self.create_subscription(
            String,
            "/imu_status",
            lambda m: self.touch("/imu_status", m.data),
            10,
        )
        self.create_subscription(
            Float32,
            "/imu_heading",
            lambda m: self.touch("/imu_heading", f"{m.data:.2f}"),
            10,
        )
        self.create_subscription(
            String,
            "/gps_pos",
            lambda m: self.touch("/gps_pos", m.data),
            10,
        )
        self.create_subscription(
            String,
            "/nav_path_data",
            lambda m: self.touch("/nav_path_data", "path received"),
            10,
        )
        self.create_subscription(
            String,
            "/vision_info",
            lambda m: self.touch("/vision_info", m.data),
            10,
        )
        self.create_subscription(
            CompressedImage,
            "/yolo_stream",
            lambda m: self.touch("/yolo_stream", f"{len(m.data)} bytes"),
            10,
        )
        self.create_subscription(
            String,
            "/vehicle_cmd",
            lambda m: self.touch("/vehicle_cmd", m.data),
            10,
        )
        self.create_subscription(
            String,
            "/vehicle_status",
            lambda m: self.touch("/vehicle_status", m.data),
            10,
        )
        self.create_subscription(
            String,
            "/set_destination",
            lambda m: self.touch("/set_destination", m.data),
            10,
        )

        self.create_timer(1.0, self.report)

    def touch(self, name, value):
        self.last_seen[name] = time.monotonic()
        self.last_value[name] = value
        self.count[name] = self.count.get(name, 0) + 1

    def report(self):
        now = time.monotonic()
        lines = []
        for name in self.names:
            if name not in self.last_seen:
                lines.append(f"{name}=NO_DATA")
                continue
            age = now - self.last_seen[name]
            lines.append(
                f"{name}={age:.1f}s n={self.count.get(name, 0)} "
                f"last={self.last_value.get(name, '')}"
            )
        self.get_logger().info(" | ".join(lines))


def main(args=None):
    rclpy.init(args=args)
    node = SensorMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
