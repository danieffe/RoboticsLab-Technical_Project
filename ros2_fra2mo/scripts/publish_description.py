#!/usr/bin/env python3
import argparse
import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class DescriptionPublisher(Node):
    def __init__(self, xacro_file: str, topic: str, repeat: int = 5):
        super().__init__('description_publisher')
        self._xacro = xacro_file
        self._topic = topic
        self._pub = self.create_publisher(String, self._topic, 10)
        urdf = ''
        try:
            urdf = subprocess.check_output(['xacro', self._xacro]).decode()
        except Exception as e:
            self.get_logger().error(f'Failed to process xacro "{self._xacro}": {e}')

        msg = String()
        msg.data = urdf

        # publish multiple times so subscribers (or gazebo) can receive it
        for _ in range(repeat):
            self._pub.publish(msg)
            self.get_logger().info(f'Published description to {self._topic}')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--xacro', required=True, help='Path to xacro file')
    parser.add_argument('--topic', required=True, help='Topic to publish the URDF string on')
    args = parser.parse_args()

    rclpy.init()
    node = DescriptionPublisher(args.xacro, args.topic)
    # give some time for publishes to go out
    rclpy.spin_once(node, timeout_sec=0.5)
    node.get_logger().info('Description publisher finished')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
