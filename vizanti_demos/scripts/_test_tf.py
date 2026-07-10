#!/usr/bin/env python3
import rclpy
import tf2_ros
import geometry_msgs.msg
import nav_msgs.msg
import math
import numpy as np

from rclpy.duration import Duration
from rclpy.node import Node

def rotation_matrix_to_quat(Rm):
    """
    Convert a 3x3 rotation matrix to a quaternion (x, y, z, w).
    Assumes a proper rotation matrix.
    """
    m00, m01, m02 = Rm[0, 0], Rm[0, 1], Rm[0, 2]
    m10, m11, m12 = Rm[1, 0], Rm[1, 1], Rm[1, 2]
    m20, m21, m22 = Rm[2, 0], Rm[2, 1], Rm[2, 2]

    trace = m00 + m11 + m22

    if trace > 0.0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (m21 - m12) * s
        y = (m02 - m20) * s
        z = (m10 - m01) * s
    elif m00 > m11 and m00 > m22:
        s = 2.0 * math.sqrt(1.0 + m00 - m11 - m22)
        w = (m21 - m12) / s
        x = 0.25 * s
        y = (m01 + m10) / s
        z = (m02 + m20) / s
    elif m11 > m22:
        s = 2.0 * math.sqrt(1.0 + m11 - m00 - m22)
        w = (m02 - m20) / s
        x = (m01 + m10) / s
        y = 0.25 * s
        z = (m12 + m21) / s
    else:
        s = 2.0 * math.sqrt(1.0 + m22 - m00 - m11)
        w = (m10 - m01) / s
        x = (m02 + m20) / s
        y = (m12 + m21) / s
        z = 0.25 * s

    quat = np.array([x, y, z, w])
    return quat / np.linalg.norm(quat)

class Particle:
    def __init__(self, i, position, velocity, node):
        self.position = np.array(position)
        self.velocity = np.array(velocity)

        self.path_publisher = node.create_publisher(nav_msgs.msg.Path, "spaceship_path_"+str(i), 10)
        self.path = nav_msgs.msg.Path()
        self.path.header.frame_id = "world"
        self.path_max_length = 200
        self.path_timer = node.get_clock().now()

class OrbitingFramesNode(Node):

    def __init__(self):
        super().__init__('test_tf')

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        self.timer_period = 1/30  # seconds
        self.G = 1.0  # Gravitational constant for simplicity
        self.mass = 1.0  # Mass of the central attractor at the origin
        self.num_frames = 10  # Number of orbiting frames

        # Initialize particles with starting positions and velocities
        self.particles = []
        for i in range(self.num_frames):
            radius = 2.0 + i * 0.5
            speed = np.sqrt(self.G * self.mass / radius)
            
            # Randomize the initial angle and plane of orbit
            theta = np.random.uniform(0, 2 * np.pi)
            phi = np.random.uniform(0, np.pi)

            position = radius * np.array([math.sin(phi) * math.cos(theta), math.sin(phi) * math.sin(theta), math.cos(phi)])
            
            # Randomize orbit direction
            orbit_direction = np.random.choice([-1, 1])
            velocity = orbit_direction * speed * np.cross(position, np.array([0, 0, 1]))
            velocity = velocity / np.linalg.norm(velocity) * speed

            self.particles.append(Particle(i, position, velocity, self))

        self.timer = self.create_timer(self.timer_period, self.publish_transforms)

    def publish_transforms(self):
        dt = self.timer_period

        for i, particle in enumerate(self.particles):
            # Calculate gravitational force towards origin
            r = np.linalg.norm(particle.position)
            force_magnitude = self.G * self.mass / (r ** 2)
            force_direction = -particle.position / r
            force = force_magnitude * force_direction

            # Update velocity and position
            particle.velocity += force * dt
            particle.position += particle.velocity * dt

            # Calculate orientation: X axis should point towards velocity vector
            x_axis = particle.velocity / np.linalg.norm(particle.velocity)
            z_axis = np.cross(x_axis, [0, 1, 0])
            if np.linalg.norm(z_axis) < 1e-6:  # Handle the special case where velocity is aligned with Y axis
                z_axis = np.cross(x_axis, [0, 0, 1])
            z_axis /= np.linalg.norm(z_axis)
            y_axis = np.cross(z_axis, x_axis)
            
            rotation_matrix = np.vstack([x_axis, y_axis, z_axis]).T
            rotation_quat = rotation_matrix_to_quat(rotation_matrix)

            # Create transform message
            transform = geometry_msgs.msg.TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = "world"
            transform.child_frame_id = f"spaceship_{i}"
            transform.transform.translation.x = particle.position[0]
            transform.transform.translation.y = particle.position[1]
            transform.transform.translation.z = particle.position[2]
            transform.transform.rotation.x = rotation_quat[0]
            transform.transform.rotation.y = rotation_quat[1]
            transform.transform.rotation.z = rotation_quat[2]
            transform.transform.rotation.w = rotation_quat[3]

            # Broadcast the transform
            self.tf_broadcaster.sendTransform(transform)

            # Track the path of the first particle
            if self.get_clock().now() - particle.path_timer > Duration(seconds=0.1):
                particle.path_timer = self.get_clock().now()
                pose = geometry_msgs.msg.PoseStamped()
                pose.header = transform.header
                pose.pose.position.x = particle.position[0]
                pose.pose.position.y = particle.position[1]
                pose.pose.position.z = particle.position[2]
                pose.pose.orientation.x = rotation_quat[0]
                pose.pose.orientation.y = rotation_quat[1]
                pose.pose.orientation.z = rotation_quat[2]
                pose.pose.orientation.w = rotation_quat[3]
                
                particle.path.poses.append(pose)
                
                if len(particle.path.poses) > particle.path_max_length:
                    particle.path.poses.pop(0)
                
                particle.path.header.stamp = self.get_clock().now().to_msg()
                particle.path_publisher.publish(particle.path)

def main(args=None):
    rclpy.init(args=args)
    node = OrbitingFramesNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()