#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf_transformations import euler_from_quaternion, quaternion_from_euler

class MapUtilsNode(Node):
    def __init__(self):
        super().__init__('map_utils_node')

        # 서비스 제공: 현재 위치 반환
        self.get_position_srv = self.create_service(
            Empty,
            'get_current_pose',
            self.handle_get_current_pose
        )

        # 서비스 제공: 맵 기준 재설정
        self.reset_map_srv = self.create_service(
            Empty,
            'reset_map_origin',
            self.handle_reset_map_origin
        )

        # tf buffer/listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info("Map Utils Node Ready")

    def handle_get_current_pose(self, request, response):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            self.get_logger().info(f"Current pose: x={t.transform.translation.x:.2f}, y={t.transform.translation.y:.2f}")
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
        return response

    def handle_reset_map_origin(self, request, response):
        """
        현재 base_link 위치를 map origin(0,0,0)으로 이동시키는 아이디어:
        1) map -> odom transform을 offset 적용
        2) 또는 slam_toolbox map reset 기능 호출
        """
        try:
            self.get_logger().info("Map origin reset requested")
            # slam_toolbox에서 제공하는 /slam_toolbox/reset 서비스 호출 가능
            client = self.create_client(Empty, '/slam_toolbox/reset')
            if not client.wait_for_service(timeout_sec=2.0):
                self.get_logger().warn("SLAM reset service not available")
                return response
            req = Empty.Request()
            client.call_async(req)
            self.get_logger().info("SLAM map reset triggered")
        except Exception as e:
            self.get_logger().error(f"Reset failed: {e}")
        return response

def main(args=None):
    rclpy.init(args=args)
    node = MapUtilsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
