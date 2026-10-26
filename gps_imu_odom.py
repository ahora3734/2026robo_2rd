# gps_imu_odom.py
#!/usr/bin/env python3
import math
import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32, String
from tf2_ros import TransformBroadcaster

class GpsImuOdom(Node):
    def __init__(self):
        super().__init__("gps_imu_odom")
        self.declare_parameter("origin_lat", 0.0)
        self.declare_parameter("origin_lon", 0.0)
        self.declare_parameter("publish_hz", 10.0)
        self.declare_parameter("gps_smoothing_alpha", 0.25)

        self.origin_lat = float(self.get_parameter("origin_lat").value)
        self.origin_lon = float(self.get_parameter("origin_lon").value)
        self.alpha = float(self.get_parameter("gps_smoothing_alpha").value)

        self.lat = None
        self.lon = None
        self.heading = 0.0

        # [핵심 수정] None으로 두어 GPS를 기다리지 않고, 0.0으로 초기화하여 SLAM 구동을 강제함
        self.x = 0.0
        self.y = 0.0
        self.filtered_x = 0.0 
        self.filtered_y = 0.0 

        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.create_subscription(String, "/gps_pos", self.gps_cb, 10)
        self.create_subscription(Float32, "/imu_heading", self.heading_cb, 10)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(
            1.0 / max(1.0, float(self.get_parameter("publish_hz").value)),
            self.publish,
        )

    def gps_cb(self, msg):
        try:
            parts = msg.data.split(",")
            if len(parts) < 2:
                return
            lat, lon = [float(v) for v in parts[:2]]
            if not (math.isfinite(lat) and math.isfinite(lon)):
                return

            if self.origin_lat == 0.0:
                self.origin_lat = lat
            if self.origin_lon == 0.0:
                self.origin_lon = lon

            lat_scale = 111320.0
            lon_scale = 111320.0 * math.cos(math.radians(self.origin_lat))

            self.x = (lon - self.origin_lon) * lon_scale
            self.y = (lat - self.origin_lat) * lat_scale

            self.filtered_x = (self.alpha * self.x + (1.0 - self.alpha) * self.filtered_x)
            self.filtered_y = (self.alpha * self.y + (1.0 - self.alpha) * self.filtered_y)
        except Exception:
            pass

    def heading_cb(self, msg):
        try:
            value = float(msg.data)
            if math.isfinite(value):
                self.heading = value % 360.0
        except Exception:
            pass

    @staticmethod
    def yaw_quaternion(yaw_deg):
        yaw = math.radians(yaw_deg)
        return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))

    def publish(self):
        try:
            now = self.get_clock().now().to_msg()
            q = self.yaw_quaternion(self.heading)

            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"
            odom.pose.pose.position.x = float(self.filtered_x)
            odom.pose.pose.position.y = float(self.filtered_y)
            odom.pose.pose.position.z = 0.0
            odom.pose.pose.orientation = q

            self.odom_pub.publish(odom)

            tf_msg = TransformStamped()
            tf_msg.header.stamp = now
            tf_msg.header.frame_id = "odom"
            tf_msg.child_frame_id = "base_link"
            tf_msg.transform.translation.x = float(self.filtered_x)
            tf_msg.transform.translation.y = float(self.filtered_y)
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation = q

            self.tf_broadcaster.sendTransform(tf_msg)
        except Exception:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = GpsImuOdom()
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