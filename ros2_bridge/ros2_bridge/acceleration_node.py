import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, ReliabilityPolicy

import websocket
import threading
import json

G_CONST = 9.81  # 1G = 9.81 m/s²


class AccelerationNode(Node):
    def __init__(self):
        super().__init__('acceleration_node')

        # WebSocket 비동기 연결
        self.ws = websocket.WebSocketApp(
            "ws://localhost:5000",
            on_open=lambda ws: self.get_logger().info("WebSocket Connected"),
            on_error=lambda ws, err: self.get_logger().warn(f"WebSocket Error: {err}"),
            on_close=lambda ws, close_status_code, close_msg: self.get_logger().info("WebSocket Closed")
        )
        self.ws_thread = threading.Thread(target=self.ws.run_forever, daemon=True)
        self.ws_thread.start()

        # ROS 구독
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

        self.create_subscription(Imu, "/imu/processed", self.cb_imu, qos_profile)
        self.get_logger().info("Acceleration Node Started")

    def cb_imu(self, msg: Imu):
        """
        /imu/processed 메시지에서 선형가속도 linear_acceleration.x, y를 G 단위로 변환
        """
        # 앞뒤 방향 (x축), 좌우 방향 (y축)
        accel_x = msg.linear_acceleration.x / G_CONST  # m/s² → G
        accel_y = msg.linear_acceleration.y / G_CONST  # m/s² → G

        # 데이터 전송용 JSON
        data = {
            "g_longitudinal": accel_x,  # 앞뒤 방향 G
            "g_lateral": accel_y       # 좌우 방향 G
        }

        # WebSocket 전송
        if self.ws.sock and self.ws.sock.connected:
            try:
                self.ws.send(json.dumps(data))
            except Exception as e:
                self.get_logger().warn(f"WebSocket send failed: {e}")


def main():
    rclpy.init()
    node = AccelerationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
