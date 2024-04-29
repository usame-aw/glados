#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

import numpy as np 
 


class LaserScanFilter(Node):

    def __init__(self):
        super().__init__('laser_scan_filter')

        self.get_logger().info("Laser Angle Filter is RUNNING")

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.filter,
            50)

        self.dist_arr = np.zeros([1080])
        
        self.publisher_ = self.create_publisher(LaserScan, '/filtered_scan', 50)


    def filter(self, prefiltered_scan):
        
        self.dist_arr = np.array(prefiltered_scan.ranges)
        self.dist_arr[650:975] = np.inf            
        prefiltered_scan.ranges = self.dist_arr.tolist()
        self.publisher_.publish(prefiltered_scan)
        # self.dist_arr[self.dist_arr>3.2] = np.inf # distance filter 
    
    
def main(args=None):
    rclpy.init(args=args)
    node = LaserScanFilter()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

