#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class DummyOdomNode(Node):
    """앞으로만 전진하는 직선 왕복 오도메트리 노드"""

    def __init__(self):
        super().__init__('dummy_odom_node')

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.timer_callback)

        # 상태 변수
        self.x = 0.0
        self.yaw = 0.0
        self.speed = 0.2           # m/s
        self.angular_speed = 1.0   # rad/s
        self.max_distance = 2.0
        self.direction = 1         # +1: +x 방향, -1: -x 방향
        self.state = "MOVE"        # MOVE 또는 TURN

        self.get_logger().info("Dummy Odom Node started — forward-only alternating motion")

    def timer_callback(self):
        dt = 0.1

        if self.state == "MOVE":
            self.x += self.direction * self.speed * dt
            if abs(self.x) >= self.max_distance:
                # 전진 완료 → 회전 준비
                self.state = "TURN"
                self.target_yaw = (self.yaw + math.pi) % (2 * math.pi)
        elif self.state == "TURN":
            # 회전 중
            delta_yaw = self.angular_speed * dt
            self.yaw += delta_yaw
            if abs((self.yaw - self.target_yaw + math.pi) % (2 * math.pi) - math.pi) < 0.05:
                # 회전 완료
                self.yaw = self.target_yaw
                self.direction *= -1
                self.state = "MOVE"

        # 오도메트리 메시지
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        odom_msg.twist.twist.linear.x = self.speed if self.state == "MOVE" else 0.0
        odom_msg.twist.twist.angular.z = self.angular_speed if self.state == "TURN" else 0.0
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
