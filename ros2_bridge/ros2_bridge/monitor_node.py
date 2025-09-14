import asyncio
import json
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, PointCloud2, NavSatFix
from nav_msgs.msg import Path
from std_msgs.msg import Float32MultiArray, Float32, Int32, UInt8
from visualization_msgs.msg import Marker
from throttle_msgs.msg import ThrottleData

import websockets
import time
import threading
from collections import deque

class MonitorNode(Node):
    def __init__(self):
        super().__init__('monitor_node')

        self.declare_parameter('ws_host', 'localhost')
        self.declare_parameter('ws_port', 5000)

        self.ws_host = self.get_parameter('ws_host').get_parameter_value().string_value
        self.ws_port = self.get_parameter('ws_port').get_parameter_value().integer_value

        self.get_logger().info(f"WebSocket Server will bind on {self.ws_host}:{self.ws_port}")

        # websocket 관련
        self.loop = asyncio.new_event_loop()
        self.ws = None
        self.thread = threading.Thread(target=self.start_loop, daemon=True)
        self.thread.start()

        # 데이터 전송 주기
        self.create_timer(1.0, self.trigger_send)

        # Hz 설정
        self.window_size = 20

        self.sensors = {
            "/usb_cam_1/image_raw": {"name": "Cam1", "min_hz": 20.0, "type": Image},
            "/usb_cam_2/image_raw": {"name": "Cam2", "min_hz": 20.0, "type": Image},
            "/ouster/points": {"name": "LiDAR", "min_hz": 10.0, "min_points": 1000, "type": PointCloud2},     # TODO
            "/cone/fused": {"name": "Fusion", "min_hz": 10.0, "min_cones": 4, "type": Image},             # TODO
            "/ublox_gps_node/fix": {"name": "GPS", "min_status": 2, "max_cov": 0.0002, "type": NavSatFix},
            "/imu/data": {"name": "IMU", "min_hz": 95, "max_hz": 105, "type": Image},                  # TODO
            "/resampled_path": {"name": "Global", "min_points": 100, "type": Image},         # TODO
            "/local_planned_path": {"name": "Local", "min_hx": 20.0, "min_length": 5, "type": Path},
            "/desired_speed_profile": {"name": "Speed", "min_array_size": 0, "type": Float32MultiArray},
            "/drivable_corridor": {"name": "Corridor", "min_valid_triangles": 1, "type": Marker},
            "/left_cone_marker": {"name": "Cones(L)", "min_hz": 10.0, "type": Marker},
            "/right_cone_marker": {"name": "Cones(R)", "min_hz": 10.0, "type": Marker},
            "/steering_command": {"name": "Steer", "min_hz": 50.0, "type": Float32},
            "/target_rpm": {"name": "RPM", "max_rpm": 2000, "type": Int32},
            "/current_speed": {"name": "CAN RX", "min_rpm": 500, "type": Int32},
            "/encoder_angle": {"name": "Error", "min_hz": 10.0, "max_error": 2.0, "type": Float32},
            "/throttle_data": {"name": "Arduino", "max_hz": 100, "type": ThrottleData},  # TODO
            "/estop": {"name": "AEB", "max_red_cones": 7, "type": UInt8},
        }

        # 각 토픽별 timestamps deque 준비
        self.timestamps = {
            topic: deque(maxlen=self.window_size) for topic in self.sensors.keys()
        }

        # 구독 생성 자동화
        self.subscribers = {}
        for topic, info in self.sensors.items():
            self.subscribers[topic] = self.create_subscription(
                info["type"],
                topic,
                lambda msg, t=topic: self._update_timestamps(t),
                10
            )

        # # TODO: 토픽명 수정
        # self.timestamps = {
        #     "/usb_cam_1/image_raw": deque(maxlen=self.window_size),  # Cam1
        #     "/usb_cam_2/image_raw": deque(maxlen=self.window_size),  # Cam2
        #     "/ouster/points": deque(maxlen=self.window_size),        # LiDAR
        #     "/cone/fused": deque(maxlen=self.window_size),           # Fusion
        #     "/imu/data": deque(maxlen=self.window_size),             # IMU
        #     "/local_planned_path": deque(maxlen=self.window_size),   # Local
        #     "/left_cone_marker": deque(maxlen=self.window_size),     # Cones(L)
        #     "/right_cone_marker": deque(maxlen=self.window_size),    # Cones(R)
        #     "/steering_command": deque(maxlen=self.window_size),     # Steer
        #     "/target_rpm": deque(maxlen=self.window_size),           # RPM
        #     "/encoder_angle": deque(maxlen=self.window_size),        # Error
        #     "/throttle_data": deque(maxlen=self.window_size),        # Arduino
        # }

        # # TODO: 토픽 수정
        # self.create_subscription(Image, "/usb_cam_1/image_raw", lambda msg: self._update_timestamps("/usb_cam_1/image_raw", 10))                 # Cam1
        # self.create_subscription(Image, "/usb_cam_2/image_raw", lambda msg: self._update_timestamps("/usb_cam_2/image_raw", 10))                 # Cam2
        # self.create_subscription(Image, "/ouster/points", lambda msg: self._update_timestamps("/ouster/points", 10))                             # LiDAR
        # self.create_subscription(Image, "/cone/fused", lambda msg:self._update_timestamps("/cone/fused", 10))                                    # Fusion
        # self.create_subscription(NavSatFix, "/ublox_gps_node/fix", lambda msg:self._update_timestamps("/ublox_gps_node/fix", 10))                # GPS
        # self.create_subscription(Image, "/imu/data", lambda msg:self._update_timestamps("/imu/data", 10))                                        # IMU
        # self.create_subscription(Image, "/resampled_path", lambda msg:self._update_timestamps("/resampled_path", 10))                            # Global
        # self.create_subscription(Path, "/local_planned_path", lambda msg:self._update_timestamps, 10)                                            # Local
        # self.create_subscription(Float32MultiArray, "/desired_speed_profile", lambda msg:self._update_timestamps("/desired_speed_profile", 10))  # Speed        
        # self.create_subscription(Marker, "/drivable_corridor", lambda msg:self._update_timestamps("/drivable_corridor", 10))                     # Corridor
        # self.create_subscription(Marker, "/left_cone_marker", lambda msg:self._update_timestamps("/left_cone_marker", 10))                       # Cones(L)
        # self.create_subscription(Marker, "/right_cone_marker", lambda msg:self._update_timestamps("/right_cone_marker", 10))                     # Cones(R)
        # self.create_subscription(Float32, "/steering_command", lambda msg:self._update_timestamps("/steering_command", 10))                      # Steer
        # self.create_subscription(Int32, "/target_rpm", lambda msg:self._update_timestamps("/target_rpm", 10))                                    # RPM
        # self.create_subscription(Int32, "/current_speed", lambda msg:self._update_timestamps("/current_speed", 10))                              # CAN RX
        # self.create_subscription(Float32, "/encoder_angle", lambda msg:self._update_timestamps("/encoder_angle", 10))                            # Error
        # self.create_subscription(ThrottleData, "/throttle_data", lambda msg:self._update_timestamps("/throttle_data", 10))                       # Arduino (.data: E-STOP, .asms: MODE)
        # self.create_subscription(UInt8, "/estop", lambda msg:self._update_timestamps("/estop", 10))                                              # AEB (cone_labeling_k)

    def start_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self.ws_handler())

    async def ws_handler(self):
        uri = f"ws://{self.ws_host}:{self.ws_port}"
        while True:
            try:
                async with websockets.connect(uri) as websocket:
                    self.get_logger().info("WebSocket connected")
                    self.ws = websocket
                    while True:
                        await asyncio.sleep(1.0)
            except Exception as e:
                self.get_logger().warn(f"WebSocket connection failed: {e}")
                await asyncio.sleep(3.0)  # 재시도 딜레이

    def trigger_send(self):
        if self.ws is not None:
            asyncio.run_coroutine_threadsafe(self.send_data(), self.loop)

    def _update_timestamps(self, topic):
        now= time.time()
        self.timestamps[topic].append(now)

    def calculate_hz(self, topic):
        times = self.timestamps[topic]

        if len(times) < 2:
            return 0.0
        
        duration = times[-1] - times[0]
        hz = (len(times) - 1) / duration if duration > 0 else 0.0
        return round(hz, 1)
    
    def get_sensor_data(self):
        data_list = []

        for topic, info in self.sensors.items():
            status = "UNKNOWN"
            value = None
            color = "gray"

            # TODO: 각 토픽별 check criteria 정의


    async def send_data(self):
        data = self.get_sensor_data()

        try:
            await self.ws.send(json.dumps(data))
        except Exception as e:
            self.get_logger().warn(f"Send failed: {e}")


def main():
    rclpy.init()
    node = MonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()