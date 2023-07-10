#!/usr/bin/env python3

# import rclpy
from typing import Dict, List, Tuple
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid


class MapPublisher(Node):
    def __init__(self, channel: str):
        super().__init__('empty_map_publisher')

        self.channel = channel
        self.pub = self.create_publisher(OccupancyGrid, self.channel, 10)

    def publish(self, msg):
        self.pub.publish(msg)
