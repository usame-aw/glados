#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
from laser_geometry import LaserProjection

class LaserScan2PointCloudConverter(Node):
    def __init__(self):

        super().__init__("Converter")
        
        self.get_logger().info("2 PC Converter is RUNNING")

        self.projector = LaserProjection()

        self.pointcloud2_publisher = self.create_publisher(
            PointCloud2, "/lidar_pc", 10
        )

        self.sense_pose_subscriber = self.create_subscription(
            LaserScan, "/filtered_scan", self.converter_cb, 10
        )

    def converter_cb(self, ls: LaserScan):
        
        pc2 = self.projector.projectLaser(ls)
        self.pointcloud2_publisher.publish(pc2)

def main(args=None):
    rclpy.init(args=args)
    node = LaserScan2PointCloudConverter()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
