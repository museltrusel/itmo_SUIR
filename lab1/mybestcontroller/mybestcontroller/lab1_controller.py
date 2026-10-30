#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
try:
    from turtlesim_msgs.msg import Pose  
except ImportError:
    from turtlesim.msg import Pose  
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

ISU_ID = '439129'


class Lab1Controller(Node):
    def __init__(self):
        super().__init__('lab1_controller')

        self.ns = self.get_namespace()
        self.is_leader = self.ns.startswith('/ns1_')
        self.get_logger().info(f'Namespace: {self.ns}, leader: {self.is_leader}')

        self.pose = None

        
        self.pose_subscriber = self.create_subscription(
            Pose, 'turtle1/pose', self.pose_callback, 10)
        self.cmd_publisher = self.create_publisher(
            Twist, 'turtle1/cmd_vel', 10)

        if self.is_leader:
            self.setup_leader()
        else:
            self.setup_follower()

        self.timer = self.create_timer(0.05, self.control_step)

    def setup_leader(self):
        # Вариант 9
        self.points = [(2.0, 3.0), (4.0, 2.0), (7.0, 1.0), (5.0, 4.0), (4.0, 2.0)]
        self.current_index = 0
        self.k_linear = 1.5
        self.k_angular = 6.0
        self.max_linear = 2.0  
        self.distance_tolerance = 0.05
        self.finished = False

        self.result_publisher = self.create_publisher(
            Float64, '/result_' + ISU_ID, 10)
        self.start_time = None
        self.elapsed = None

    def setup_follower(self):
        
        leader_ns = '/ns1_' + self.ns.split('_', 1)[1]
        leader_pose_topic = leader_ns + '/turtle1/pose'
        self.get_logger().info(f'Following {leader_pose_topic}')

        self.leader_pose = None
        self.leader_pose_subscriber = self.create_subscription(
            Pose, leader_pose_topic, self.leader_pose_callback, 10)

        
        self.k_linear = 5.0
        self.k_angular = 12.0
        self.distance_tolerance = 0.1

    def pose_callback(self, msg):
        self.pose = msg

    def leader_pose_callback(self, msg):
        self.leader_pose = msg

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def errors(self, target_x, target_y):
        dx = target_x - self.pose.x
        dy = target_y - self.pose.y
        distance = math.sqrt(dx ** 2 + dy ** 2)
        angle_error = self.normalize_angle(math.atan2(dy, dx) - self.pose.theta)
        return distance, angle_error

    def stop(self):
        self.cmd_publisher.publish(Twist())

    def publish_result(self):
        msg = Float64()
        msg.data = self.elapsed
        self.result_publisher.publish(msg)

    def control_step(self):
        if self.pose is None:
            self.get_logger().info('Waiting for turtle pose...')
            return

        if self.is_leader:
            self.leader_step()
        else:
            self.follower_step()

    def leader_step(self):
        if self.finished:
            return

        if self.start_time is None:
            self.start_time = self.get_clock().now()
            self.get_logger().info('Start timing')

        target_x, target_y = self.points[self.current_index]
        distance, angle_error = self.errors(target_x, target_y)

        if distance < self.distance_tolerance:
            self.stop()
            self.get_logger().info(
                f'Point {self.current_index + 1}/{len(self.points)} '
                f'({target_x}, {target_y}) reached')
            self.current_index += 1

            if self.current_index >= len(self.points):
                self.finished = True
                self.elapsed = (self.get_clock().now()
                                - self.start_time).nanoseconds / 1e9
                self.get_logger().info(
                    f'All points reached! Time: {self.elapsed:.2f} s')
                self.publish_result()
                
                self.result_timer = self.create_timer(1.0, self.publish_result)
            return

        # P-регулятор из ПР №1
        cmd = Twist()
        if abs(angle_error) > 0.2:
            cmd.linear.x = 0.0
        else:
            cmd.linear.x = min(self.k_linear * distance, self.max_linear)
        cmd.angular.z = self.k_angular * angle_error
        self.cmd_publisher.publish(cmd)

    def follower_step(self):
        if self.leader_pose is None:
            return

        
        distance, angle_error = self.errors(self.leader_pose.x, self.leader_pose.y)

        if distance < self.distance_tolerance:
            self.stop()
            return

        
        cmd = Twist()
        cmd.linear.x = self.k_linear * distance * max(math.cos(angle_error), 0.0)
        cmd.angular.z = self.k_angular * angle_error
        self.cmd_publisher.publish(cmd)


def main():
    rclpy.init()
    node = Lab1Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()