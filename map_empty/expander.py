#!/usr/bin/env python3

import rclpy
import os

from .MapSubscriber import MapSubscriber

MAP_TOPIC = os.environ.get('MAP_TOPIC')
if not MAP_TOPIC:
    print('MAP_TOPIC environment variable not set.')
    print('Cannot read map to be expanded as a result.')
    print('Exiting.')
    exit(1)


def main(args=None):
    rclpy.init(args=args)

    sub = MapSubscriber(
        input_channel=MAP_TOPIC,
        output_channel=f'{MAP_TOPIC}/map_expanded',
    )

    rclpy.spin(sub)

    # destroy the node when it is not used anymore
    sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    print('Starting ...')
    main()
