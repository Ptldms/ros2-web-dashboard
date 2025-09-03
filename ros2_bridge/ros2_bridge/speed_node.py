import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

import json
import websocket

'''
/cmd/speed, /current_speed 구독
'''

class SpeedNode(Node):
    def __init__(self):
        super().__init__('speed_node')
        self.ws = websocket.WebSocket()
        self.ws.connect("ws://localhost:5000")

        # 처음에는 None (값이 들어오기 전 상태)
        self.cmd_speed = None
        self.current_speed = None

        self.create_subscription(Float32, '/cmd/speed', self.cb_cmdSpeed, 10)
        self.create_subscription(Float32, '/current_speed', self.cb_crtSpeed, 10)

    def cb_cmdSpeed(self, msg: Float32):
        self.cmd_speed = msg.data
        self.send_speed_data()

    def cb_crtSpeed(self, msg: Float32):
        self.current_speed = msg.data
        self.send_speed_data()

    def send_speed_data(self):
        data = {}
        if self.cmd_speed is not None:
            data["cmd_speed"] = self.cmd_speed
        if self.current_speed is not None:
            data["current_speed"] = self.current_speed

        # 두 토픽 다 아직 안 들어왔으면 skip
        if not data:
            return

        try:
            self.ws.send(json.dumps(data))
        except Exception as e:
            self.get_logger().warn(f"WebSocket send failed: {e}")


def main():
    rclpy.init()
    node = SpeedNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()