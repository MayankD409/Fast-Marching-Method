import rclpy
from rclpy.node import Node
import numpy as np
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from fmm import fast_marching_method, prune_path
import time
import threading

class PurePursuitController(Node):
    def __init__(self, path, lookahead_distance, start_time):
        super().__init__('pure_pursuit_controller')
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.t = 0.0
        self.initial_x = None
        self.initial_y = None
        self.initial_theta = None
        self.start_x = 0.0
        self.start_y = 1.5

        self.path = path
        self.translated_path = []
        self.current_goal_index = 0

        self.lookahead_distance = lookahead_distance

        self.max_linear_velocity = 1.1  # Tweak
        self.max_angular_velocity = 3.0 # Tweak

        self.sub_vel = self.create_subscription(
            Odometry,
            '/odom',
            self.update_position,
            10)

        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_robot)

        self.start_time = start_time
        self.stop_requested = False
        self.i = 0
        self.first_odom = True
        self.p=True

    def create_inverse_transformation_matrix(self, x, y, theta):
        # Create the correct inverse transformation matrix
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
        return np.array([
            [cos_theta, sin_theta, 0, -x*cos_theta - y*sin_theta],
            [-sin_theta, cos_theta, 0, x*sin_theta - y*cos_theta],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

    def update_position(self, msg):
        self.i +=1

        if self.initial_x is None or self.initial_y is None or self.initial_theta is None:
            self.initial_x = msg.pose.pose.position.x
            self.initial_y = msg.pose.pose.position.y
            orientation_q = msg.pose.pose.orientation
            _, _, self.initial_theta = euler_from_quaternion(
                [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
            self.translate_path()

            self.x = 0
            self.y = 0
            self.theta = 0

        orientation_q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        
        self.x = (msg.pose.pose.position.x - self.initial_x)
        self.y = (msg.pose.pose.position.y - self.initial_y)
        self.theta = yaw - self.initial_theta

        print(f"Initial position -> X: {self.initial_x:.2f}, Y: {self.initial_y:.2f}, Yaw (theta): {self.initial_theta:.2f}")
        print(f"Current position -> X: {self.x:.2f}, Y: {self.y:.2f}, Yaw (theta): {self.theta:.2f}")
    
    def translate_path(self):
        for node_x, node_y in self.path:
            translated_x = (node_x/10)
            translated_y = ((node_y/10) -1.5)
            self.translated_path.append((translated_x, translated_y))

    def control_robot(self):
        if self.current_goal_index >= len(self.translated_path):
            self.pub_vel.publish(Twist())
            elapsed_time = time.time() - self.start_time
            print(f"Final node reached! Time taken: {elapsed_time:.2f} seconds")
            exit()

        goal_x, goal_y = self.find_lookahead_point()
        print(f"Present goals -> X: {goal_x}, Y: {goal_y}")

        distance_to_goal = math.sqrt((goal_x - self.x) ** 2 + (goal_y - self.y) ** 2)
        desired_angle = math.atan2(goal_y - self.y, goal_x - self.x)
        angle_difference = desired_angle - self.theta
        print("desired_angle", desired_angle)
        print("angle_difference", angle_difference)

        # Normalize angle difference to range [-pi, pi]
        angle_difference = (angle_difference + math.pi) % (2 * math.pi) - math.pi

        if distance_to_goal < 0.05:
            print(f"Reached goal at: ({self.x:.2f}, {self.y:.2f})")
            self.current_goal_index += 1
            return

        angle_penalty = max(0, 1 - 2 * abs(angle_difference) / math.pi)
        linear_velocity = min(self.max_linear_velocity * angle_penalty, distance_to_goal)
        # linear_velocity = min(self.max_linear_velocity, distance_to_goal)
        angular_velocity = 2.5 * angle_difference  # Tweak

        velocity_msg = Twist()
        velocity_msg.linear.x = linear_velocity
        velocity_msg.angular.z = (max(min(angular_velocity, self.max_angular_velocity), -self.max_angular_velocity))

        self.pub_vel.publish(velocity_msg)

    def find_lookahead_point(self):
        print("Translated Path:", self.translated_path)
        for index in range(self.current_goal_index, len(self.translated_path)):
            goal_x, goal_y = self.translated_path[index]
            distance = math.sqrt((goal_x - self.x) ** 2 + (goal_y - self.y) ** 2)
            if distance >= self.lookahead_distance:
                self.current_goal_index = index
                return goal_x, goal_y

        return self.translated_path[-1]
    
    def stop_robot(self):
        """Publishes a zero velocity message to stop the robot."""
        stop_msg = Twist()
        self.pub_vel.publish(stop_msg)
        print("Robot stopped.")
        self.destroy_node()

def input_thread(controller):
    while True:
        if input("Press 'q' and Enter to stop the robot: ") == 'q':
            controller.stop_requested = True
            break

def main(args=None):
    start_time = time.time()
    rclpy.init(args=args)
    start_position = (0, 15)
    goal_position = (60, 14)
    clearance = 0.6  # Tweak
    robot_radius = 1.9  # Tweak
    grid_width, grid_height = 61, 30
    x_path, y_path = fast_marching_method(start_position, goal_position, clearance, robot_radius, grid_width, grid_height)
    x_path, y_path = prune_path(x_path, y_path, robot_radius, clearance)
    path = list(zip(x_path, y_path))
    print("Planned!!!")
    print("length:", len(path))
    print("Pruned Coordinates of the path:", (x_path, y_path))
    lookahead_distance = 0.4  # Tweak
    controller = PurePursuitController(path, lookahead_distance, start_time)
    # Start the input thread
    threading.Thread(target=input_thread, args=(controller,), daemon=True).start()

    try:
        while rclpy.ok():
            if controller.stop_requested:
                controller.stop_robot()
                break
            rclpy.spin_once(controller)
    finally:
        # Properly destroy the node and shutdown rclpy
        if rclpy.ok():
            controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()




