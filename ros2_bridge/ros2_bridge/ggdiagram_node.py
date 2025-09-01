import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
import numpy as np

G_CONST = 9.81

class GGDiagram(Node):
    def __init__(self):
        super().__init__('ggdiagram node')
        self.sub = self.create_subscription(
            Imu, 'imu_processed', cb_imu, 10
        )