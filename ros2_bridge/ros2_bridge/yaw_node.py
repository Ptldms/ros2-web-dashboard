import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

import json
import websocket
import math

'''
/odometry/filtered 구독 -> 쿼터니언을 오일러 값으로 변환 후 yaw 값 계산
'''

class YawNode(Node):
    def __init__(self):
        super().__init__('yaw_node')
        self.ws = websocket.WebSocket()
        self.ws.connect("ws://localhost:5000")

        self.create_subscription(Odometry, '/odometry/filtered', self.cb_odom, 10)

    def cb_odom(self, msg: Odometry):
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        yaw = math.atan2(2.0*(qw*qz + qx*qy), 1 - 2.0*(qy*qy + qz*qz))

        data = {"raw": yaw}

        try:
            self.ws.send(json.dumps(data))
        except Exception as e:
            self.get_logger().warn(f"WebSocket send failed: {e}")

def main():
    rclpy.init()
    node = YawNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()