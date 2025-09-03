import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

import json
import websocket

'''
/cmd/steer, /current_steer? 구독
TODO: 현재 조향각 토픽 추가
'''

class SteerNode(Node):
    def __init__(self):
        super().__init__('steer_node')
        self.ws = websocket.WebSocket()
        self.ws.connect("ws://localhost:5000")

        self.cmd_steer = None
        # self.current_steer = None

        self.create_subscription(Float32, '/cmd/steer', self.cb_cmdSteer, 10)
        # self.create_subscription(Float32, '/current_steer', self.cb_crtSteer, 10)

    def cb_cmdSteer(self, msg: Float32):
        self.cmd_steer = msg.data
        self.send_steer_data()

    # def cb_crtSteer(self, msg: Float32):
    #     self.current_steer = msg.data
    #     self.send_steer_data()

    def send_steer_data(self):
        data = {}
        if self.cmd_steer is not None:
            data["cmd_steer"] = self.cmd_steer
        # if self.current_steer is not None:
        #     data["current_steer"] = self.current_steer

        # 두 토픽 다 아직 안 들어왔으면 skip
        if not data:
            return

        try:
            self.ws.send(json.dumps(data))
        except Exception as e:
            self.get_logger().warn(f"WebSocket send failed: {e}")


def main():
    rclpy.init()
    node = SteerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()