#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
import yaml
import os

class IiwaPoseSaver(Node):
    def __init__(self):
        super().__init__('iiwa_pose_saver')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.subscription = self.create_subscription(
            PoseStamped,
            'fra2mo/aruco_single/pose',
            self.aruco_callback,
            10)

        self.output_path = os.path.join(
            os.getenv('HOME'), 'ros2_ws/src/ros2_fra2mo/config/iiwa_goal.yaml'
        )
        
        self.iiwa_found = False
        self.get_logger().info('Node Saver Ready: Searching iiwa ...')

    def aruco_callback(self, msg):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 
                msg.header.frame_id, 
                rclpy.time.Time())

            pose_in_map = do_transform_pose(msg.pose, transform)

            self.save_to_yaml(pose_in_map)
            
            if not self.iiwa_found:
                self.get_logger().info(f'!!! IIWA FOUND !!! Pose saved in: {self.output_path}')
                self.get_logger().info(f'Map pose: X={pose_in_map.position.x:.2f}, Y={pose_in_map.position.y:.2f}')
                self.iiwa_found = True

        except Exception as e:
            self.get_logger().warn(f'Error during the tranformation... {str(e)}')

    def save_to_yaml(self, pose):
        data = {
            'iiwa_goal': {
                'x': float(pose.position.x),
                'y': float(pose.position.y),
                'z': 0.0,
                'yaw': 0.0 
            }
        }
        
        os.makedirs(os.path.dirname(self.output_path), exist_ok=True)
        with open(self.output_path, 'w') as outfile:
            yaml.dump(data, outfile, default_flow_style=False)

        self.get_logger().info(
            f"FILE SAVED! Position updated: X={data['iiwa_goal']['x']:.3f}, Y={data['iiwa_goal']['y']:.3f}"
        )



def main(args=None):
    rclpy.init(args=args)
    node = IiwaPoseSaver()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()