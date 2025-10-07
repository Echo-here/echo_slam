#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster
import numpy as np


class DummyOdomNode(Node):
    """가짜 오도메트리 + 360도 원형 라이다 퍼블리셔"""

    def __init__(self):
        super().__init__('dummy_odom_node')

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.timer_callback)

        # 상태 변수
        self.x = 0.0
        self.yaw = 0.0
        self.speed = 0.2
        self.angular_speed = 1.0
        self.max_distance = 2.0
        self.direction = 1
        self.state = "MOVE"

        self.get_logger().info("Dummy Odom + LIDAR started — simulating motion with circular scan (/scan, frame=laser)")

    def timer_callback(self):
        dt = 0.1

        # 이동 로직
        if self.state == "MOVE":
            self.x += self.direction * self.speed * dt
            if abs(self.x) >= self.max_distance:
                self.state = "TURN"
                self.target_yaw = (self.yaw + math.pi) % (2 * math.pi)
        elif self.state == "TURN":
            self.yaw += self.angular_speed * dt
            if abs((self.yaw - self.target_yaw + math.pi) % (2 * math.pi) - math.pi) < 0.05:
                self.yaw = self.target_yaw
                self.direction *= -1
                self.state = "MOVE"

        # 오도메트리 메시지
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        self.odom_pub.publish(odom_msg)

        # TF (odom → base_link)
        tf = TransformStamped()
        tf.header.stamp = odom_msg.header.stamp
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x = self.x
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = 0.0
        tf.transform.rotation = odom_msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(tf)

        # ✅ TF (base_link → laser)
        tf2 = TransformStamped()
        tf2.header.stamp = odom_msg.header.stamp
        tf2.header.frame_id = 'base_link'
        tf2.child_frame_id = 'laser'
        tf2.transform.translation.x = 0.0
        tf2.transform.translation.y = 0.0
        tf2.transform.translation.z = 0.2  # 약간 띄운 높이
        tf2.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(tf2)

        # ✅ 360도 라이다 스캔 (반경 2.0m 원형)
        scan_msg = LaserScan()
        scan_msg.header.stamp = odom_msg.header.stamp
        scan_msg.header.frame_id = 'laser'

        scan_msg.angle_min = -math.pi
        scan_msg.angle_max = math.pi
        scan_msg.angle_increment = math.radians(1.0)  # 1도 간격
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 0.1
        scan_msg.range_min = 0.05
        scan_msg.range_max = 10.0

        num_readings = int((scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment)

        # 원형 벽: 반경 2.0m
        ranges = np.ones(num_readings) * 2.0
        # 약간의 노이즈 추가
        ranges += np.random.normal(0, 0.02, num_readings)
        scan_msg.ranges = ranges.tolist()

        self.scan_pub.publish(scan_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DummyOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
