#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import tf_transformations
import random
import math

class OdomNoiseSim(Node):
    def __init__(self):
        super().__init__('odom_noise_sim')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.publisher = self.create_publisher(
            Odometry,
            '/odom',
            10)
        self.initial_noise_added = False
        self.get_logger().info("Odometry noise simulator started.")

    def odom_callback(self, msg):
        if not self.initial_noise_added:

            initial_x_noise = random.uniform(-5, 5)
            initial_y_noise = random.uniform(-5, 5)
            msg.pose.pose.position.x += initial_x_noise
            msg.pose.pose.position.y += initial_y_noise
            

            orientation_q = msg.pose.pose.orientation
            quaternion = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            euler = tf_transformations.euler_from_quaternion(quaternion)
            roll, pitch, yaw = euler
            yaw_noise = random.uniform(-math.pi/2, math.pi/2)
            yaw += yaw_noise
            new_quat = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
            msg.pose.pose.orientation = Quaternion(x=new_quat[0], y=new_quat[1], z=new_quat[2], w=new_quat[3])
            
            self.initial_noise_added = True

        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    odom_noise_sim = OdomNoiseSim()
    rclpy.spin(odom_noise_sim)
    odom_noise_sim.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    main()
