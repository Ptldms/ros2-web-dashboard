import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, ReliabilityPolicy

import threading
import websocket
import json
import math

G_CONST = 9.81  # 1G = 9.81 m/s²


class VehicleWSNode(Node):
    def __init__(self):
        super().__init__('vehicle_ws_node')

        # WebSocketApp 비동기 실행
        self.ws = websocket.WebSocketApp(
            "ws://192.168.0.34:5000",  # TODO: ip 주소 변경
            on_open=lambda ws: self.get_logger().info("WebSocket Connected"),
            on_error=lambda ws, err: self.get_logger().warn(f"WebSocket Error: {err}"),
            on_close=lambda ws, close_status_code, close_msg: self.get_logger().info("WebSocket Closed")
        )
        self.ws_thread = threading.Thread(target=self.ws.run_forever, daemon=True)
        self.ws_thread.start()

        # 전송 데이터 초기화
        self.data = {
            "cmd_speed": None,
            "current_speed": None,
            "cmd_steer": None,
            "yaw": None,
            "g_longitudinal": None,
            "g_lateral": None
        }

        # ROS 구독자
        self.create_subscription(Float32, '/cmd/speed', self.cb_cmd_speed, 10)
        self.create_subscription(Float32, '/current_speed', self.cb_current_speed, 10)
        self.create_subscription(Float32, '/cmd/steer', self.cb_cmd_steer, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self.cb_odometry, 10)

        # IMU 구독자 (선형 가속도)
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(Imu, '/imu/processed', self.cb_imu, qos_profile)

        # 30Hz(33ms) 주기로 데이터 전송
        self.create_timer(0.033, self.send_data)

    # --- ROS Callbacks ---
    def cb_cmd_speed(self, msg):
        self.data["cmd_speed"] = msg.data

    def cb_current_speed(self, msg):
        self.data["current_speed"] = msg.data

    def cb_cmd_steer(self, msg):
        self.data["cmd_steer"] = msg.data

    def cb_odometry(self, msg):
        qx, qy, qz, qw = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
        # Yaw 계산 (Z축 기준)
        self.data["yaw"] = math.atan2(2.0 * (qw * qz + qx * qy),
                                      1 - 2.0 * (qy * qy + qz * qz))

    def cb_imu(self, msg: Imu):
        """
        /imu/processed 메시지에서 선형가속도 linear_acceleration.x, y를 G 단위로 변환
        """
        # 앞뒤 방향 (x축), 좌우 방향 (y축)
        self.data["g_longitudinal"] = msg.linear_acceleration.x / G_CONST
        self.data["g_lateral"] = msg.linear_acceleration.y / G_CONST

    # --- WebSocket 전송 ---
    def send_data(self):
        if self.ws.sock and self.ws.sock.connected:
            payload = {k: v for k, v in self.data.items() if v is not None}
            if payload:
                try:
                    self.ws.send(json.dumps(payload))
                except Exception as e:
                    self.get_logger().warn(f"Send failed: {e}")


def main():
    rclpy.init()
    node = VehicleWSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
