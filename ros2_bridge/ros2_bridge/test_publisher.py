import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, PointCloud2, NavSatFix
from nav_msgs.msg import Path
from std_msgs.msg import Float32MultiArray, Float32, Int32, UInt8
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped, Point

import numpy as np
import struct

class TestPublisherNode(Node):
    def __init__(self):
        super().__init__('test_publisher')

        # 카메라 이미지 (Cam1, Cam2)
        self.cam1_pub = self.create_publisher(Image, "/usb_cam_1/image_raw", 10)
        self.cam2_pub = self.create_publisher(Image, "/usb_cam_2/image_raw", 10)

        # 라이다
        self.lidar_pub = self.create_publisher(PointCloud2, "/ouster/points", 10)

        # 융합
        self.fusion_pub = self.create_publisher(Marker, "/cone/fused", 10)

        # GPS
        self.gps_pub = self.create_publisher(NavSatFix, "/ublox_gps_node/fix", 10)

        # IMU
        self.imu_pub = self.create_publisher(Image, "/imu/data", 10)  # dummy image

        # Path
        self.global_pub = self.create_publisher(Path, "/resampled_path", 10)
        self.local_pub = self.create_publisher(Path, "/local_planned_path", 10)

        # Speed
        self.speed_pub = self.create_publisher(Float32MultiArray, "/desired_speed_profile", 10)

        # Corridor
        self.corridor_pub = self.create_publisher(Marker, "/drivable_corridor", 10)

        # Cones
        self.conesL_pub = self.create_publisher(Marker, "/left_cone_marker", 10)
        self.conesR_pub = self.create_publisher(Marker, "/right_cone_marker", 10)

        # Steering, RPM, Speed
        self.steer_pub = self.create_publisher(Float32, "/steering_command", 10)
        self.rpm_pub = self.create_publisher(Int32, "/target_rpm", 10)
        self.canRX_pub = self.create_publisher(Int32, "/current_speed", 10)

        # Encoder
        self.encoder_pub = self.create_publisher(Float32, "/encoder_angle", 10)

        # AEB
        self.aeb_pub = self.create_publisher(UInt8, "/estop", 10)

        # Timer 주기 (모든 토픽 발행)
        self.timer = self.create_timer(0.05, self.publish_all)  # 20Hz 이상 보장

        # steering 값 기록
        self.angle = 10.0

    def publish_all(self):
        # 1. Camera
        self.cam1_pub.publish(Image())
        self.cam2_pub.publish(Image())

        # 2. Lidar (10000 points, 충분히 조건 충족)
        pc = PointCloud2()
        pc.height = 1
        pc.width = 10000
        pc.point_step = 16
        pc.row_step = pc.width * pc.point_step
        pc.data = b'\x00' * pc.row_step
        self.lidar_pub.publish(pc)

        # 3. Fusion (10 cones)
        marker = Marker()
        marker.points = [Point(x=0.0, y=0.0, z=0.0) for _ in range(10)]
        self.fusion_pub.publish(marker)

        # 4. GPS (RTK + 낮은 covariance)
        gps = NavSatFix()
        gps.status.status = 2
        gps.position_covariance = [0.0001] * 9
        self.gps_pub.publish(gps)

        # 5. IMU
        self.imu_pub.publish(Image())

        # 6. Global path (200 poses)
        path = Path()
        for i in range(200):
            pose = PoseStamped()
            path.poses.append(pose)
        self.global_pub.publish(path)

        # 7. Local path (10 poses, 길이 충분히 확보)
        path2 = Path()
        for i in range(10):
            pose = PoseStamped()
            pose.pose.position.x = i * 1.0
            path2.poses.append(pose)
        self.local_pub.publish(path2)

        # 8. Speed profile
        sp = Float32MultiArray()
        sp.data = [5.0, 10.0, 15.0]
        self.speed_pub.publish(sp)

        # 9. Corridor
        cor = Marker()
        cor.type = 11
        cor.points = [Point(x=float(x), y=0.0, z=0.0) for x in [0, 5, 10]]
        self.corridor_pub.publish(cor)

        # 10. Cones
        markerL = Marker()
        markerL.points = [Point(x=0.0, y=0.0, z=0.0) for _ in range(5)]
        self.conesL_pub.publish(markerL)

        markerR = Marker()
        markerR.points = [Point(x=0.0, y=0.0, z=0.0) for _ in range(5)]
        self.conesR_pub.publish(markerR)

        # 11. Steering
        self.steer_pub.publish(Float32(data=self.angle))

        # 12. RPM
        self.rpm_pub.publish(Int32(data=1500))

        # 13. CAN RX speed
        self.canRX_pub.publish(Int32(data=1000))

        # 14. Encoder (angle과 차이 1.0 → OK)
        self.encoder_pub.publish(Float32(data=self.angle - 1.0))

        # 15. AEB (red cones 3 < 7 → OK)
        self.aeb_pub.publish(UInt8(data=3))


def main():
    rclpy.init()
    node = TestPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
