#!/usr/bin/env python3
import os
import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
import paho.mqtt.client as mqtt
from tf_transformations import euler_from_quaternion, quaternion_from_euler

# 환경 변수
MQTT_BROKER = os.getenv("MQTT_BROKER", "localhost")
MQTT_PORT = int(os.getenv("MQTT_PORT", 1883))
MQTT_TOPIC_RECORD = os.getenv("MQTT_TOPIC_RECORD", "echo/record")
MQTT_TOPIC_GO = os.getenv("MQTT_TOPIC_GO", "echo/go")
MQTT_TOPIC_TELEOP = os.getenv("MQTT_TOPIC_TELEOP", "echo/teleop")


class MQTTGoalMemoryNode(Node):
    def __init__(self):
        super().__init__('mqtt_goal_memory_node')
        
        # 기존 goal publisher
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)
        
        # 추가된 teleop publisher
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.memory = {}  # {name: PoseStamped}

        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        self.mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
        self.mqtt_client.loop_start()

        self.get_logger().info(f"Node Ready. MQTT Broker={MQTT_BROKER}:{MQTT_PORT}")

    def on_connect(self, client, userdata, flags, rc):
        self.get_logger().info(f"Connected to MQTT broker with code {rc}")
        client.subscribe(MQTT_TOPIC_RECORD)
        client.subscribe(MQTT_TOPIC_GO)
        client.subscribe(MQTT_TOPIC_TELEOP)  # 🔥 추가됨

    def on_message(self, client, userdata, msg):
        try:
            payload = msg.payload.decode('utf-8')

            # ------------------------------
            # 🔥 TELEOP 처리 (wasd)
            # ------------------------------
            if msg.topic == MQTT_TOPIC_TELEOP:
                cmd = payload.strip().lower()

                twist = Twist()
                LIN = 0.25
                ANG = 0.8

                if cmd in ["w", "forward"]:
                    twist.linear.x = LIN
                elif cmd in ["s", "back"]:
                    twist.linear.x = -LIN
                elif cmd in ["a", "left"]:
                    twist.angular.z = ANG
                elif cmd in ["d", "right"]:
                    twist.angular.z = -ANG
                elif cmd == "q":  # 전진 + 좌회전
                    twist.linear.x = LIN
                    twist.angular.z = ANG
                elif cmd == "e":  # 전진 + 우회전
                    twist.linear.x = LIN
                    twist.angular.z = -ANG
                elif cmd in ["x", "stop", "0"]:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                else:
                    self.get_logger().warn(f"Unknown teleop command: {cmd}")
                    return

                self.cmd_vel_pub.publish(twist)
                self.get_logger().info(f"[Teleop] cmd='{cmd}' → lin={twist.linear.x}, ang={twist.angular.z}")
                return

            # ------------------------------
            # 🔥 기존 JSON 기반 record/go 처리
            # ------------------------------
            data = json.loads(payload)
            name = data.get("name")

            if not name:
                self.get_logger().warn("No name provided in MQTT message")
                return

            if msg.topic == MQTT_TOPIC_RECORD:
                t = self.tf_lookup()
                if t is None:
                    self.get_logger().warn("Cannot record pose: TF lookup failed")
                    return
                self.memory[name] = t
                self.get_logger().info(
                    f"Recorded pose for '{name}': x={t.pose.position.x:.2f}, y={t.pose.position.y:.2f}"
                )

            elif msg.topic == MQTT_TOPIC_GO:
                if name not in self.memory:
                    self.get_logger().warn(f"No recorded pose for '{name}'")
                    return
                pose = self.memory[name]
                pose.header.stamp = self.get_clock().now().to_msg()
                self.goal_pub.publish(pose)
                self.get_logger().info(
                    f"Published goal for '{name}': x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f}"
                )

        except Exception as e:
            self.get_logger().error(f"Error processing MQTT message: {e}")

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
