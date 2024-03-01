#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_msgs.msg import ParticleCloud
from geometry_msgs.msg import PoseArray, Pose
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

import random

class ParticleCloudToPoseArrayNode(Node):

    def __init__(self):
        super().__init__('particle_cloud_to_pose_array')
        
        qos_profile = QoSProfile(
            depth=1,
            reliability=0
        )
        
        self.subscription = self.create_subscription(
            ParticleCloud,
            '/particle_cloud',
            self.particle_cloud_callback,
            qos_profile
        )
        
        self.publisher = self.create_publisher(
            PoseArray,
            '/poses',
            qos_profile
        )

    def particle_cloud_callback(self, msg):
        selected_indices = random.sample(msg.particles, 20)
        
        pose_array = PoseArray()
        pose_array.header = msg.header
        
        for particle in selected_indices:
            pose = Pose()
            pose = particle.pose
            pose_array.poses.append(pose)
        
        self.publisher.publish(pose_array)

def main(args=None):
    rclpy.init(args=args)
    node = ParticleCloudToPoseArrayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()