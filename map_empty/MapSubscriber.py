#!/usr/bin/env python3

from map_expand.MapPublisher import MapPublisher

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid


class MapSubscriber(Node):
    def __init__(
        self,
        input_channel: str,
        output_channel: str,
    ):
        super().__init__('simulation_map_expander_subscriber')

        self.input_channel = input_channel
        self.output_channel = output_channel

        self.sub = self.create_subscription(
            OccupancyGrid, self.input_channel, self._callback, 10
        )
        self.pub = MapPublisher(self.output_channel)

    def _callback(self, data: OccupancyGrid):
        self.get_logger().info(
            f'Received map:\n\theader: {data.header}\n\tinfo: {data.info}\n\tdata length: {len(data.data)}'
        )

        # increase map area
        data.info.width *= 20
        data.info.height *= 20

        # adapt number of data entries accordingly
        data.data = [-1] * (data.info.width * data.info.height)

        # offset map so it is centered
        data.info.origin.position.x -= (
            data.info.width * data.info.resolution / 2
        )
        data.info.origin.position.y -= (
            data.info.height * data.info.resolution / 2
        )

        self.pub.publish(data)
        self.get_logger().info(
            f'Published map:\n\theader: {data.header}\n\tinfo: {data.info}\n\tdata length: {len(data.data)}'
        )
