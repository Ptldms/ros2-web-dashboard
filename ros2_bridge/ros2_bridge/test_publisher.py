import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, PointCloud2, NavSatFix, Imu
from nav_msgs.msg import Path
from std_msgs.msg import Float32MultiArray, Float32, Int32, UInt8
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped, Point

import numpy as np
import random


class TestPublisherNode(Node):
    def __init__(self):
        super().__init__('test_publisher')

        # Publishers
        self.cam1_pub = self.create_publisher(Image, "/usb_cam_1/image_raw", 10)
        self.cam2_pub = self.create_publisher(Image, "/usb_cam_2/image_raw", 10)
        self.lidar_pub = self.create_publisher(PointCloud2, "/ouster/points", 10)
        self.fusion_pub = self.create_publisher(Marker, "/cone/fused", 10)
        self.gps_pub = self.create_publisher(NavSatFix, "/ublox_gps_node/fix", 10)
        self.imu_pub = self.create_publisher(Imu, "/imu/data", 10)
        self.global_pub = self.create_publisher(Path, "/resampled_path", 10)
        self.local_pub = self.create_publisher(Path, "/local_planned_path", 10)
        self.speed_pub = self.create_publisher(Float32MultiArray, "/desired_speed_profile", 10)
        self.corridor_pub = self.create_publisher(Marker, "/drivable_corridor", 10)
        self.conesL_pub = self.create_publisher(Marker, "/left_cone_marker", 10)
        self.conesR_pub = self.create_publisher(Marker, "/right_cone_marker", 10)
        self.steer_pub = self.create_publisher(Float32, "/steering_command", 10)
        self.rpm_pub = self.create_publisher(Int32, "/target_rpm", 10)
        self.canRX_pub = self.create_publisher(Int32, "/current_speed", 10)
        self.encoder_pub = self.create_publisher(Float32, "/encoder_angle", 10)
        self.aeb_pub = self.create_publisher(UInt8, "/estop", 10)

        # Timer
        self.timer = self.create_timer(0.05, self.publish_all)  # 20Hz 이상 보장

    def publish_all(self):
        # 1. Camera (더미 이미지)
        self.cam1_pub.publish(Image())
        self.cam2_pub.publish(Image())

        # 2. Lidar (랜덤 포인트 수)
        pc = PointCloud2()
        pc.height = 1
        pc.width = random.randint(1000, 20000)
        pc.point_step = 16
        pc.row_step = pc.width * pc.point_step
        pc.data = b'\x00' * pc.row_step
        self.lidar_pub.publish(pc)

        # 3. Fusion (cones 개수 랜덤)
        marker = Marker()
        marker.points = [Point(
            x=random.uniform(-10, 10),
            y=random.uniform(-10, 10),
            z=0.0
        ) for _ in range(random.randint(0, 20))]
        self.fusion_pub.publish(marker)

        # 4. GPS (랜덤 위치 + covariance)
        gps = NavSatFix()
        gps.status.status = random.choice([-1, 0, 1, 2])
        gps.latitude = random.uniform(-90, 90)
        gps.longitude = random.uniform(-180, 180)
        gps.altitude = random.uniform(0, 1000)
        gps.position_covariance = [random.uniform(0.0001, 5.0) for _ in range(9)]
        self.gps_pub.publish(gps)

        # 5. IMU (랜덤 가속도 + 각속도)
        imu = Imu()
        imu.linear_acceleration.x = random.uniform(-20, 20)
        imu.linear_acceleration.y = random.uniform(-20, 20)
        imu.linear_acceleration.z = random.uniform(-20, 20)
        imu.angular_velocity.x = random.uniform(-5, 5)
        imu.angular_velocity.y = random.uniform(-5, 5)
        imu.angular_velocity.z = random.uniform(-5, 5)
        self.imu_pub.publish(imu)

        # 6. Global path (랜덤 길이)
        path = Path()
        for i in range(random.randint(50, 300)):
            pose = PoseStamped()
            pose.pose.position.x = random.uniform(-50, 50)
            pose.pose.position.y = random.uniform(-50, 50)
            path.poses.append(pose)
        self.global_pub.publish(path)

        # 7. Local path
        path2 = Path()
        for i in range(random.randint(5, 20)):
            pose = PoseStamped()
            pose.pose.position.x = random.uniform(0, 20)
            pose.pose.position.y = random.uniform(-5, 5)
            path2.poses.append(pose)
        self.local_pub.publish(path2)

        # 8. Speed profile
        sp = Float32MultiArray()
        sp.data = [random.uniform(0, 30) for _ in range(3)]
        self.speed_pub.publish(sp)

        # 9. Corridor
        cor = Marker()
        cor.type = 11
        cor.points = [Point(
            x=random.uniform(0, 20),
            y=random.uniform(-5, 5),
            z=0.0
        ) for _ in range(3)]
        self.corridor_pub.publish(cor)

        # 10. Cones
        markerL = Marker()
        markerL.points = [Point(
            x=random.uniform(0, 10),
            y=random.uniform(0, 5),
            z=0.0
        ) for _ in range(random.randint(0, 10))]
        self.conesL_pub.publish(markerL)

        markerR = Marker()
        markerR.points = [Point(
            x=random.uniform(0, 10),
            y=random.uniform(-5, 0),
            z=0.0
        ) for _ in range(random.randint(0, 10))]
        self.conesR_pub.publish(markerR)

        # 11. Steering
        self.steer_pub.publish(Float32(data=random.uniform(-30.0, 30.0)))

        # 12. RPM
        self.rpm_pub.publish(Int32(data=random.randint(0, 2500)))

        # 13. CAN RX speed
        self.canRX_pub.publish(Int32(data=random.randint(400, 800)))

        # 14. Encoder
        self.encoder_pub.publish(Float32(data=random.uniform(-30.0, 30.0)))

        # 15. AEB (빨간 콘 갯수)
        self.aeb_pub.publish(UInt8(data=random.randint(0, 10)))


def main():
    rclpy.init()
    node = TestPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
