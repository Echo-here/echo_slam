#!/usr/bin/env python3
import os
import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from action_msgs.msg import GoalStatusArray
import paho.mqtt.client as mqtt
from tf_transformations import euler_from_quaternion, quaternion_from_euler


# ------------------------------------------------------------
# 환경변수
# ------------------------------------------------------------
MQTT_BROKER = os.getenv("MQTT_BROKER", "localhost")
MQTT_PORT = int(os.getenv("MQTT_PORT", 443))

MQTT_TOPIC_RECORD = os.getenv("MQTT_TOPIC_RECORD", "echo/record")
MQTT_TOPIC_GO = os.getenv("MQTT_TOPIC_GO", "echo/go")
MQTT_TOPIC_TELEOP = os.getenv("MQTT_TOPIC_TELEOP", "echo/teleop")

MQTT_TOPIC_GET = os.getenv("MQTT_TOPIC_GET", "echo/record/get")
MQTT_TOPIC_VALUE = os.getenv("MQTT_TOPIC_VALUE", "echo/record/value")

MQTT_TOPIC_NAV = os.getenv("MQTT_TOPIC_NAV", "echo/nav")
MQTT_TOPIC_NAV_GET = os.getenv("MQTT_TOPIC_NAV_GET", "echo/nav/get")


# ------------------------------------------------------------
# 메인 노드
# ------------------------------------------------------------
class MQTTGoalMemoryNode(Node):
    def __init__(self):
        super().__init__('mqtt_goal_memory_node')

        # 현재 Nav2 상태 (idle / success / fail)
        self.current_nav_state = "idle"

        # pose 저장 딕셔너리
        self.memory = {}

        # ROS Publishers
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # ROS Subscribers
        self.nav_status_sub = self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/_action/status',
            self.nav2_status_callback,
            10
        )

        # MQTT 설정
        self.mqtt_client = mqtt.Client(transport="websockets")
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        self.mqtt_client.tls_set()
        self.mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
        self.mqtt_client.loop_start()

        self.get_logger().info("MQTT Goal Memory Node initialized.")


    # ------------------------------------------------------------
    # MQTT connect
    # ------------------------------------------------------------
    def on_connect(self, client, userdata, flags, rc):
        client.subscribe(MQTT_TOPIC_RECORD)
        client.subscribe(MQTT_TOPIC_GO)
        client.subscribe(MQTT_TOPIC_TELEOP)
        client.subscribe(MQTT_TOPIC_GET)
        client.subscribe(MQTT_TOPIC_NAV_GET)
        self.get_logger().info("MQTT connected & subscribed.")


    # ------------------------------------------------------------
    # MQTT message handler
    # ------------------------------------------------------------
    def on_message(self, client, userdata, msg):
        try:
            payload = msg.payload.decode('utf-8')

            # NAV 상태 재요청 처리
            if msg.topic == MQTT_TOPIC_NAV_GET:
                self.mqtt_client.publish(MQTT_TOPIC_NAV, self.current_nav_state)
                self.get_logger().info(f"[Nav/Get] → {self.current_nav_state}")
                return

            # Teleop 처리
            if msg.topic == MQTT_TOPIC_TELEOP:
                self.handle_teleop(payload)
                return

            # Record 목록 요청
            if msg.topic == MQTT_TOPIC_GET:
                self.publish_record_keys()
                return

            # JSON 데이터 처리
            data = json.loads(payload)
            name = data.get("name")

            if not name:
                self.get_logger().warn("JSON has no 'name'")
                return

            # Record 저장
            if msg.topic == MQTT_TOPIC_RECORD:
                pose = self.tf_lookup()
                if pose:
                    self.memory[name] = pose
                    self.get_logger().info(f"[Record] '{name}' saved")
                return

            # 저장된 위치로 이동
            elif msg.topic == MQTT_TOPIC_GO:
                if name not in self.memory:
                    self.get_logger().warn(f"No record for '{name}'")
                    return

                pose = self.memory[name]
                pose.header.stamp = self.get_clock().now().to_msg()
                self.goal_pub.publish(pose)
                self.get_logger().info(f"[Go] Moving to '{name}'")
                return

        except Exception as e:
            self.get_logger().error(f"MQTT message error: {e}")


    # ------------------------------------------------------------
    # Nav2 상태 감지 (idle / success / fail)
    # ------------------------------------------------------------
    def nav2_status_callback(self, msg: GoalStatusArray):
        if not msg.status_list:
            # Goal 없음 → idle
            if self.current_nav_state != "idle":
                self.current_nav_state = "idle"
                self.mqtt_client.publish(MQTT_TOPIC_NAV, "idle")
                self.get_logger().info("[Nav2] idle")
            return

        status = msg.status_list[-1].status

        # SUCCEEDED
        if status == 4:
            if self.current_nav_state != "success":
                self.current_nav_state = "success"
                self.mqtt_client.publish(MQTT_TOPIC_NAV, "success")
                self.get_logger().info("[Nav2] success")
            return

        # ABORTED or CANCELED → fail
        if status in [5, 6]:
            if self.current_nav_state != "fail":
                self.current_nav_state = "fail"
                self.mqtt_client.publish(MQTT_TOPIC_NAV, "fail")
                self.get_logger().info("[Nav2] fail")
            return

        # 그 외 상태는 이동 중 → 방송하지 않음


    # ------------------------------------------------------------
    # teleop 제어
    # ------------------------------------------------------------
    def handle_teleop(self, cmd):
        twist = Twist()
        cmd = cmd.strip().lower()

        LIN = 0.25
        ANG = 0.8

        mapping = {
            "w": (LIN, 0),
            "s": (-LIN, 0),
            "a": (0, ANG),
            "d": (0, -ANG),
            "q": (LIN, ANG),
            "e": (LIN, -ANG),
            "x": (0, 0),
            "stop": (0, 0),
            "0": (0, 0),
        }

        if cmd in mapping:
            twist.linear.x, twist.angular.z = mapping[cmd]
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info(f"[Teleop] {cmd}")
        else:
            self.get_logger().warn(f"Unknown teleop command: {cmd}")


    # ------------------------------------------------------------
    # Record 목록 MQTT 발행
    # ------------------------------------------------------------
    def publish_record_keys(self):
        keys = list(self.memory.keys())
        msg = json.dumps(keys)
        self.mqtt_client.publish(MQTT_TOPIC_VALUE, msg)
        self.get_logger().info(f"[Record/Get] → {keys}")


    # ------------------------------------------------------------
    # TF lookup (map → base_link)
    # ------------------------------------------------------------
    def tf_lookup(self):
        try:
            import tf2_ros
            tf_buffer = tf2_ros.Buffer()
            tf_listener = tf2_ros.TransformListener(tf_buffer, self)

            t = tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())

            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = t.transform.translation.x
            pose.pose.position.y = t.transform.translation.y
            pose.pose.position.z = t.transform.translation.z

            q = t.transform.rotation
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            q2 = quaternion_from_euler(0, 0, yaw)
            pose.pose.orientation.x = q2[0]
            pose.pose.orientation.y = q2[1]
            pose.pose.orientation.z = q2[2]
            pose.pose.orientation.w = q2[3]

            return pose

        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return None


# ------------------------------------------------------------
# main
# ------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = MQTTGoalMemoryNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
