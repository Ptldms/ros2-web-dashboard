import asyncio
import json
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, PointCloud2, NavSatFix, Imu
from nav_msgs.msg import Path
from std_msgs.msg import Float32MultiArray, Float32, Int32, UInt8
from visualization_msgs.msg import Marker
# from throttle_msgs.msg import ThrottleData

import websockets
import time
import threading
from collections import deque
import math

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

        # sensor 기본 설정 (topic별 모니터링 조건)
        self.sensor_config = {
            "/usb_cam_1/image_raw": {
                "name": "Cam1",
                "min_hz": 20.0,
            },
            "/usb_cam_2/image_raw": {
                "name": "Cam2",
                "min_hz": 20.0,
            },
            "/ouster/points": {
                "name": "LiDAR",
                "min_hz": 10.0,
                "min_points": 1000,
            },
            "/cone/fused": {
                "name": "Fusion",
                "min_hz": 10.0,
                "min_cones": 4,
            },
            "/ublox_gps_node/fix": {
                "name": "GPS",
                "min_status": 2,
                "max_cov": 0.0002,
            },
            "/imu/data": {
                "name": "IMU",
                "min_hz": 95.0,
                "max_hz": 105.0,
            },
            "/resampled_path": {
                "name": "Global",
                "min_points": 100,
            },
            "/local_planned_path": {
                "name": "Local",
                "min_hz": 20.0,
                "min_length": 5,
            },
            "/desired_speed_profile": {
                "name": "Speed",
                "min_array_size": 0,
            },
            "/drivable_corridor": {
                "name": "Corridor",
                "min_valid_triangles": 1,
            },
            "/left_cone_marker": {
                "name": "Cones(L)",
                "min_hz": 10.0,
            },
            "/right_cone_marker": {
                "name": "Cones(R)",
                "min_hz": 10.0,
            },
            "/steering_command": {
                "name": "Steer",
                "min_hz": 50.0,
            },
            "/target_rpm": {
                "name": "RPM",
                "max_hz": 100,
                "max_rpm": 2000,
            },
            "/current_speed": {
                "name": "CAN RX",
                "min_rpm": 500,
            },
            "/encoder_angle": {
                "name": "Error",
                "min_hz": 10.0,
                "max_error": 2.0,
            },
            "/throttle_data": {
                "name": "Arduino",
                "max_hz": 100.0,
            },
            "/estop": {
                "name": "AEB",
                "max_red_cones": 7,
            },
        }

        # 실시간 센서 데이터값
        self.sensor_data = {}

        # 각 토픽별 timestamps deque 준비
        self.timestamps = {
            topic: deque(maxlen=self.window_size) for topic in self.sensor_config.keys()
        }

        # TODO: 토픽 수정
        self.create_subscription(Image, "/usb_cam_1/image_raw", self.cam1_cb, 10)                 # Cam1
        self.create_subscription(Image, "/usb_cam_2/image_raw", self.cam2_cb, 10)                 # Cam2
        self.create_subscription(PointCloud2, "/ouster/points", self.lidar_cb, 10)                      # LiDAR
        self.create_subscription(Marker, "/cone/fused", self.fusion_cb, 10)                        # Fusion
        self.create_subscription(NavSatFix, "/ublox_gps_node/fix", self.gps_cb, 10)               # GPS
        self.create_subscription(Imu, "/imu/data", self.imu_cb, 10)                             # IMU
        self.create_subscription(Path, "/resampled_path", self.global_cb, 10)                    # Global
        self.create_subscription(Path, "/local_planned_path", self.local_cb, 10)                  # Local
        self.create_subscription(Float32MultiArray, "/desired_speed_profile", self.speed_cb, 10)  # Speed        
        self.create_subscription(Marker, "/drivable_corridor", self.corridor_cb, 10)              # Corridor
        self.create_subscription(Marker, "/left_cone_marker", self.conesL_cb, 10)                 # Cones(L)
        self.create_subscription(Marker, "/right_cone_marker", self.conesR_cb, 10)                # Cones(R)
        self.create_subscription(Float32, "/steering_command", self.steer_cb, 10)                 # Steer
        self.create_subscription(Int32, "/target_rpm", self.rpm_cb, 10)                           # RPM
        self.create_subscription(Int32, "/current_speed", self.canRX_cb, 10)                      # CAN RX
        self.create_subscription(Float32, "/encoder_angle", self.error_cb, 10)                    # Error
        # self.create_subscription(ThrottleData, "/throttle_data", self.arduino_cb, 10)             # Arduino (.data: E-STOP, .asms: MODE)
        self.create_subscription(UInt8, "/estop", self.aeb_cb, 10)                                # AEB (cone_labeling_k)

    def cam1_cb(self, msg):
        topic = "/usb_cam_1/image_raw"
        self._update_timestamps(topic)

        hz = self.calculate_hz(topic)
        config = self.sensor_config[topic]

        is_ok = hz > config["min_hz"]

        self.sensor_data[topic] = {
            "status": "GO" if is_ok else "NO-GO",
            "value": hz,
            "color": "lime" if is_ok else "red"
        }
    
    def cam2_cb(self, msg):
        topic = "/usb_cam_2/image_raw"
        self._update_timestamps(topic)

        hz = self.calculate_hz(topic)
        config = self.sensor_config[topic]

        is_ok = hz > config["min_hz"]

        self.sensor_data[topic] = {
            "status": "GO" if is_ok else "NO-GO",
            "value": hz,
            "color": "lime" if is_ok else "red"
        }
    
    def lidar_cb(self, msg):
        topic = "/ouster/points"
        self._update_timestamps(topic)

        hz = self.calculate_hz(topic)

        if msg.height > 1:
            num_points = msg.width * msg.height
        else:
            num_points = len(msg.data) // msg.point_step

        config = self.sensor_config[topic]

        is_ok = hz > config["min_hz"] and num_points > config["min_points"]

        self.sensor_data[topic] = {
            "status": "GO" if is_ok else "NO-GO",
            "value": hz,
            "color": "lime" if is_ok else "red"
        }

    def fusion_cb(self, msg):
        topic = "/cone/fused"
        self._update_timestamps(topic)

        hz = self.calculate_hz(topic)

        # TODO: Marker 타입이라고 가정
        if hasattr(msg, 'points'):
            num_cones = len(msg.points)
        else:
            num_cones = 0
        
        config = self.sensor_config[topic]

        is_ok = hz > config["min_hz"] and num_cones > config["min_cones"]

        self.sensor_data[topic] = {
            "status": "GO" if is_ok else "NO-GO",
            "value": num_cones,
            "color": "lime" if is_ok else "red"
        }
    
    def gps_cb(self, msg):
        topic = "/ublox_gps_node/fix"
        self._update_timestamps(topic)

        hz = self.calculate_hz(topic)
        status_code = msg.status.status
        cov = msg.position_covariance[0] if len(msg.position_covariance) > 0 else 999.0
        config = self.sensor_config[topic]

        is_ok = status_code >= config["min_status"] and cov < config["max_cov"]

        if status_code >= 2:
            status_text = "RTK"
        elif status_code == 1:
            status_text = "DGPS"
        elif status_code == 0:
            status_text = "GPS"
        else:
            status_text = "No"
        
        self.sensor_data[topic] = {
            "status": status_text if is_ok else "NO-GO",
            "value": f"{status_text}",
            "color": "lime" if is_ok else "red"
        }

    def imu_cb(self, msg):
        topic = "/imu/data"
        self._update_timestamps(topic)

        hz = self.calculate_hz(topic)
        config = self.sensor_config[topic]

        is_ok = config["min_hz"] <= hz <= config["max_hz"]

        self.sensor_data[topic] = {
            "status": "GO" if is_ok else "NO-GO",
            "value": hz,
            "color": "lime" if is_ok else "red"
        }

    def global_cb(self, msg):
        topic = "/resampled_path"
        self._update_timestamps(topic)

        # TODO: Path 타입이라고 가정
        points = len(msg.poses)
        config = self.sensor_config[topic]

        is_ok = points > config["min_points"]

        self.sensor_data[topic] = {
            "status": "GO" if is_ok else "NO-GO",
            "value": points,
            "color": "lime" if is_ok else "red"
        }

    def local_cb(self, msg):
        topic = "/local_planned_path"
        self._update_timestamps(topic)

        hz = self.calculate_hz(topic)
        path_length = 0.0
        
        if len(msg.poses) > 1:
            for i in range(1, len(msg.poses)):
                prev_pose = msg.poses[i-1].pose.position
                curr_pose = msg.poses[i].pose.position
                
                dx = curr_pose.x - prev_pose.x
                dy = curr_pose.y - prev_pose.y
                dz = curr_pose.z - prev_pose.z

                distance = math.sqrt(dx*dx + dy*dy + dz*dz)
                path_length += distance
        
        config = self.sensor_config[topic]

        is_ok = hz > config["min_hz"] and path_length > config["min_length"]

        self.sensor_data[topic] = {
            "status": "GO" if is_ok else "NO-GO",
            "value": hz,
            "color": "lime" if is_ok else "red"
        }

    def speed_cb(self, msg):
        topic = "/desired_speed_profile"
        self._update_timestamps(topic)

        array_size = len(msg.data)
        config = self.sensor_config[topic]

        is_ok = array_size > config["min_array_size"]

        if array_size > 0:
            max_speed = max(msg.data)
        else:
            max_speed = 0.0

        self.sensor_data[topic] = {
            "status": "GO" if is_ok else "NO-GO",
            "value": max_speed,
            "color": "lime" if is_ok else "red"
        }

    # TODO: check criteria 수정
    def corridor_cb(self, msg):
        topic = "/drivable_corridor"
        self._update_timestamps(topic)

        if hasattr(msg, 'points') and msg.type == 11:
            valid_triangles = len(msg.points) // 3
            
            if len(msg.points) > 0:
                x_coords = [point.x for point in msg.points]
                width = max(x_coords) - min(x_coords)
            else:
                width = 0.0
        else:
            valid_triangles = 0
            width = 0.0
    
        config = self.sensor_config[topic]
        
        is_ok = valid_triangles >= config["min_valid_triangles"]
        
        self.sensor_data[topic] = {
            "status": "GO" if is_ok else "NO-GO",
            "value": width,
            "color": "lime" if is_ok else "red"
        }

    def conesL_cb(self, msg):
        topic = "/left_cone_marker"
        self._update_timestamps(topic)

        hz = self.calculate_hz(topic)
        config = self.sensor_config[topic]

        is_ok = hz > config["min_hz"]

        if hasattr(msg, 'points'):
            num_cones = len(msg.points)
        else:
            num_cones = 0

        self.sensor_data[topic] = {
            "status": "GO" if is_ok else "NO-GO",
            "value": num_cones,
            "color": "lime" if is_ok else "red"
        }
    
    def conesR_cb(self, msg):
        topic = "/right_cone_marker"
        self._update_timestamps(topic)

        hz = self.calculate_hz(topic)
        config = self.sensor_config[topic]

        is_ok = hz > config["min_hz"]

        if hasattr(msg, 'points'):
            num_cones = len(msg.points)
        else:
            num_cones = 0

        self.sensor_data[topic] = {
            "status": "GO" if is_ok else "NO-GO",
            "value": num_cones,
            "color": "lime" if is_ok else "red"
        }

    def steer_cb(self, msg):
        topic = "/steering_command"
        self._update_timestamps(topic)

        hz = self.calculate_hz(topic)
        config = self.sensor_config[topic]

        is_ok = hz > config["min_hz"]

        angle = msg.data
        self.last_steering_cmd = angle

        self.sensor_data[topic] = {
            "status": "GO" if is_ok else "NO-GO",
            "value": angle,
            "color": "lime" if is_ok else "red"
        }
    
    def rpm_cb(self, msg):
        topic = "/target_rpm"
        self._update_timestamps(topic)

        hz = self.calculate_hz(topic)
        rpm = msg.data
        config = self.sensor_config[topic]

        is_ok = hz == config["max_hz"] and rpm <= config["max_rpm"]

        self.sensor_data[topic] = {
            "status": "GO" if is_ok else "NO-GO",
            "value": rpm,
            "color": "lime" if is_ok else "red"
        }

    # TODO: check criteria 수정
    def canRX_cb(self, msg):
        topic = "/current_speed"
        self._update_timestamps(topic)

        rpm = msg.data
        config = self.sensor_config[topic]

        is_ok = rpm >= config["min_rpm"]

        self.sensor_data[topic] = {
            "status": "GO" if is_ok else "NO-GO",
            "value": rpm,
            "color": "lime" if is_ok else "red"
        }

    def error_cb(self, msg):
        topic = "/encoder_angle"
        self._update_timestamps(topic)

        hz = self.calculate_hz(topic)
        encoder_angle = msg.data

        if hasattr(self, "last_steering_cmd"):
            error = self.last_steering_cmd - encoder_angle
        else:
            error: None

        config = self.sensor_config[topic]

        is_ok = hz > config["min_hz"] and error < config["max_error"]
        
        self.sensor_data[topic] = {
            "status": "GO" if is_ok else "NO-GO",
            "value": error,
            "color": "lime" if is_ok else "red"
        }

    def arduino_cb(self, msg):
        topic = "/throttle_data"
        self._update_timestamps(topic)

        hz = self.calculate_hz(topic)
        config = self.sensor_config[topic]

        is_ok = hz == config["max_hz"]

        self.sensor_data[topic] = {
            "status": "GO" if is_ok else "NO-GO",
            "value": hz,
            "color": "lime" if is_ok else "red"
        }

    def aeb_cb(self, msg):
        topic = "/estop"
        self._update_timestamps(topic)

        red_cones_count = msg.data
        config = self.sensor_config[topic]
        
        is_ok = red_cones_count < config["max_red_cones"]
        
        self.sensor_data[topic] = {
            "status": "GO" if is_ok else "NO-GO",
            "value": red_cones_count,
            "color": "lime" if is_ok else "red"
        }

    def start_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self.ws_handler())

    async def ws_handler(self):
        uri = f"ws://{self.ws_host}:{self.ws_port}"
        while True:
            try:
                async with websockets.connect(
                    uri,
                    ping_interval=20,   # 20초마다 ping
                    ping_timeout=10     # pong 없으면 10초 안에 timeout
                ) as websocket:
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
        data = {}
        for topic, config in self.sensor_config.items():
            name = config["name"]
            value = self.sensor_data.get(topic, {
                "status": "NO DATA",
                "value": None,
                "color": "gray"
            })

            # value가 숫자면 소수점 첫째자리까지 반올림
            if isinstance(value.get("value"), (int, float)):
                value["value"] = round(value["value"], 1)

            data[name] = value
        return data

    async def send_data(self):
        data = self.get_sensor_data()

        try:
            # ws가 None이 아니고 연결 상태이면 전송 시도
            if self.ws is None:
                self.get_logger().warn("WebSocket is None, skip sending")
                return

            # JSON으로 직렬화 불가능한 값이 들어있을 수 있으니 안전하게 변환
            try:
                payload = json.dumps([
                    {"name": key, **value} for key, value in data.items()
                ])
            except TypeError:
                # 직렬화 불가 항목을 문자열로 변환해서 보내기 (간단한 폴백)
                safe_data = {}
                for k, v in data.items():
                    try:
                        json.dumps(v)
                        safe_data[k] = v
                    except TypeError:
                        safe_data[k] = {
                            "status": str(v.get("status", "")) if isinstance(v, dict) else str(v),
                            "value": str(v.get("value", "")) if isinstance(v, dict) else str(v),
                            "color": str(v.get("color", "gray")) if isinstance(v, dict) else "gray"
                        }
                payload = json.dumps(safe_data)

            await self.ws.send(payload)

        except Exception as e:
            # 경고 로그 남기고 ws 객체 초기화해 재접속 시도하게 함
            self.get_logger().warn(f"Send failed: {e}")
            try:
                # 안전하게 ws 참조 해제
                self.ws = None
            except Exception:
                pass

def main():
    rclpy.init()
    node = MonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()