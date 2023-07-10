#!/usr/bin/env python3

from rclpy.node import Node
from typing import Dict, List, Tuple
import rclpy
import os

from nav_msgs.msg import OccupancyGrid

# from .MapPublisher import MapPublisher

OUTPUT_CHANNEL = os.environ.get('OUTPUT_CHANNEL')
if not OUTPUT_CHANNEL:
    OUTPUT_CHANNEL = '/map_empty'


class MapPublisher(Node):
    def __init__(self, channel: str):
        super().__init__('empty_map_publisher')
        self.channel = channel
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.pub = self.create_publisher(OccupancyGrid, self.channel, 10)
        self.data = self.new_empty()

    def new_empty(self) -> OccupancyGrid:
        data = OccupancyGrid()

        data.header.frame_id = 'world'

        # increase map area
        data.info.width = 200
        data.info.height = 200

        # offset map so it is centered
        data.info.origin.position.x -= (
            data.info.width * data.info.resolution / 2
        )
        data.info.origin.position.y -= (
            data.info.height * data.info.resolution / 2
        )

        # adapt number of data entries accordingly
        data.data = [-1] * (data.info.width * data.info.height)

        return data

    def timer_callback(self, msg):
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    pub = MapPublisher(OUTPUT_CHANNEL)
    rclpy.spin(pub)
    pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    print('Starting ...')
    main()
    print('Done.')
