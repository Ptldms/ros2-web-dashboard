import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import websocket
import json

"""
/ublox_gps_node/fix 구독 -> 위도/경도/공분산 추출
"""

class GPSNode(Node):
    def __init__(self):
        super().__init__('gps_node')

        self.ws = websocket.WebSocket()
        self.ws.connect("ws://localhost:5000")

        self.create_subscription(NavSatFix, 'ublox_gps_node', self.cb_gps, 10)

    def cb_gps(self, msg: NavSatFix):
        lat = msg.latitude
        lon = msg.longitude
        cov00 = msg.position_covariance[0] if len(msg.position_covariance) > 0 else 9999.0

        data = {
            "lat": lat,
            "lon": lon,
            "cov": cov00
        }

        try:
            self.ws.send(json.dumps(data))
        except Exception as e:
            self.get_logger().warn(f"WebSocket send failed: {e}")

def main():
    rclpy.init()
    node = GPSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()