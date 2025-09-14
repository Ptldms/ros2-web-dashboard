import asyncio
import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import websockets
import time
import threading
import random  # test용

class MonitorNode(Node):
    def __init__(self):
        super().__init__('monitor_node')

        self.declare_parameter('ws_host', 'localhost')
        self.declare_parameter('ws_port', 5000)

        self.ws_host = self.get_parameter('ws_host').get_parameter_value().string_value
        self.ws_port = self.get_parameter('ws_port').get_parameter_value().integer_value

        self.get_logger().info(f"WebSocket Server will bind on {self.ws_host}:{self.ws_port}")

        #asyncio 루프 별도 thread에서 실행
        self.loop = asyncio.new_event_loop()
        self.ws = None
        self.thread = threading.Thread(target=self.start_loop, daemon=True)
        self.thread.start()

        # 1초 주기로 데이터 전송
        self.create_timer(1.0, self.trigger_send)

    #     self.min_hz = 20.0
    #     self.window_size = 20  # 최근 20개 메시지 기준으로 Hz 계산

    #     # 시간 기록용 버퍼
    #     self.timestamps = {
    #         "/usb_cam_1/image_raw": [],
    #         "/usb_cam_2/image_raw": []
    #     }

    #     self.create_subscription(Image, "/usb_cam_1/image_raw", self.cam1_callback, 10)
    #     self.create_subscription(Image, "/usb_cam_2/image_raw", self.cam2_callback, 10)

    # def cam1_callback(self, msg):
    #     self._update_timestamps("/usb_cam_1/image_raw")

    # def cam2_callback(self, msg):
    #     self._update_timestamps("/usb_cam_2/image_raw")

    # def _update_timestamps(self, topic):
    #     now = time.time()
    #     self.timestamps[topic].append(now)
    #     if len(self.timestamps[topic]) > self.window_size:
    #         self.timestamps[topic].pop(0)

    # def check_topic_hz(self, topic_name):
    #     times = self.timestamps[topic_name]
    #     if len(times) < 2:
    #         return {"hz": 0.0, "status": "NO-GO"}

    #     duration = times[-1] - times[0]
    #     hz = (len(times) - 1) / duration if duration > 0 else 0.0
    #     hz = round(hz, 1)
    #     status = "GO" if hz > self.min_hz else "NO-GO"
    #     return {"hz": hz, "status": status}


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
        """ROS Timer에서 호출 → asyncio loop에서 실행"""
        if self.ws is not None:
            asyncio.run_coroutine_threadsafe(self.send_data(), self.loop)

    async def send_data(self):
        cam1_freq = round(random.uniform(0, 30), 1)
        cam2_freq = round(random.uniform(0, 30), 1)

        cam1_status = "GO" if cam1_freq > 20 else "NO-GO"
        cam2_status = "GO" if cam2_freq > 20 else "NO-GO"

        data = [
            {
                "name": "Cam1",
                "status": cam1_status,
                "color": "lime" if cam1_status == "GO" else "red",
                "freq": cam1_freq,
                "unit": "Hz"
            },
            {
                "name": "Cam2",
                "status": cam2_status,
                "color": "lime" if cam2_status == "GO" else "red",
                "freq": cam2_freq,
                "unit": "Hz"
            }
        ]

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