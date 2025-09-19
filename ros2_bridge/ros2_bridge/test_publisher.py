import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, PointCloud2, NavSatFix, Imu
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Float32, UInt8
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped, Point, Twist, Pose, PoseWithCovariance, TwistWithCovariance

import numpy as np
import random
import math


class TestPublisherNode(Node):
    def __init__(self):
        super().__init__('test_publisher')

        # Publishers
        self.cam1_pub = self.create_publisher(Image, "/usb_cam_1/image_raw", 10)
        self.cam2_pub = self.create_publisher(Image, "/usb_cam_2/image_raw", 10)
        self.lidar_pub = self.create_publisher(PointCloud2, "/ouster/points", 10)
        self.fusion_pub = self.create_publisher(Marker, "/cone/fused/ukf", 10)
        self.gps_pub = self.create_publisher(NavSatFix, "/ublox_gps_node/fix", 10)
        self.imu_pub = self.create_publisher(Imu, "/imu/processed", 10)
        self.global_pub = self.create_publisher(Path, "/resampled_path", 10)
        self.local_pub = self.create_publisher(Path, "/local_planned_path", 10)
        self.planning_speed_pub = self.create_publisher(Float32, "/cmd/speed", 10)
        self.planning_steer_pub = self.create_publisher(Float32, "/cmd/steer", 10)
        self.control_steer_pub = self.create_publisher(Float32, "/ctrl/steer", 10)
        self.control_speed_pub = self.create_publisher(Float32, "/ctrl/speed", 10)
        self.canRX_pub = self.create_publisher(Float32, "/current_speed", 10)
        self.aeb_pub = self.create_publisher(UInt8, "/estop", 10)
        self.odometry_pub = self.create_publisher(Odometry, "/odometry/filtered", 10)

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

        # 8. Speed (Planning)
        self.planning_speed_pub.publish(Float32(data=random.uniform(0.0, 30.0)))

        # 9. Steer (Planning)
        self.planning_steer_pub.publish(Float32(data=random.uniform(-30.0, 30.0)))

        # 10. Steer (Control)
        self.control_steer_pub.publish(Float32(data=random.uniform(-30.0, 30.0)))

        # 11. Speed (Control)
        self.control_speed_pub.publish(Float32(data=random.uniform(0.0, 30.0)))

        # 12. CAN RX speed
        self.canRX_pub.publish(Float32(data=random.uniform(400.0, 800.0)))

        # 13. AEB (빨간 콘 갯수)
        self.aeb_pub.publish(UInt8(data=random.randint(0, 10)))

        # 16. Odometry (차량 위치 + 자세)
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"
        
        # 위치 (랜덤)
        odom.pose.pose.position.x = random.uniform(-100, 100)
        odom.pose.pose.position.y = random.uniform(-100, 100)
        odom.pose.pose.position.z = 0.0
        
        # 자세 (Quaternion) - 랜덤 yaw 각도
        yaw = random.uniform(-math.pi, math.pi)
        qw = math.cos(yaw / 2.0)
        qz = math.sin(yaw / 2.0)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        
        # 속도 (랜덤)
        odom.twist.twist.linear.x = random.uniform(-10.0, 10.0)
        odom.twist.twist.linear.y = random.uniform(-2.0, 2.0)
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = random.uniform(-1.0, 1.0)

        # Covariance 매트릭스 (6x6 = 36개 요소)
        # [x, y, z, roll, pitch, yaw] 순서
        pose_cov = [0.0] * 36
        twist_cov = [0.0] * 36
        
        # 대각선 요소만 설정 (불확실성)
        pose_cov[0] = random.uniform(0.01, 1.0)   # x 불확실성
        pose_cov[7] = random.uniform(0.01, 1.0)   # y 불확실성
        pose_cov[35] = random.uniform(0.01, 0.1)  # yaw 불확실성
        
        twist_cov[0] = random.uniform(0.01, 0.5)   # vx 불확실성
        twist_cov[7] = random.uniform(0.01, 0.5)   # vy 불확실성
        twist_cov[35] = random.uniform(0.01, 0.1)  # vyaw 불확실성
        
        odom.pose.covariance = pose_cov
        odom.twist.covariance = twist_cov
        
        self.odometry_pub.publish(odom)

def main():
    rclpy.init()
    node = TestPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
