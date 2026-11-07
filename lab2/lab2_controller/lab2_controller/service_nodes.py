import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
try:
    from turtlesim_msgs.msg import Pose
except ImportError:
    from turtlesim.msg import Pose

from lab2_interfaces.srv import A439129


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class WaypointService(Node):
    targets = ((2.0, 3.0), (4.0, 2.0), (7.0, 1.0), (5.0, 4.0), (4.0, 2.0))
    reached_tolerance = 0.1

    def __init__(self):
        super().__init__('waypoint_service')
        self.index = 0
        self.srv = self.create_service(A439129, 'get_waypoint_error', self.callback)

    def callback(self, request, response):
        if self.index >= len(self.targets):
            response.distance = 0.0
            response.angle = 0.0
            response.success = True
            response.complete = True
            return response

        x, y = self.targets[self.index]
        dx = x - request.x_current
        dy = y - request.y_current
        response.distance = math.hypot(dx, dy)
        response.angle = math.atan2(dy, dx)
        response.success = response.distance < self.reached_tolerance
        response.complete = False
        if response.distance < self.reached_tolerance:
            self.get_logger().info(f'Waypoint {self.index + 1} reached: ({x}, {y})')
            self.index += 1
        return response


class WaypointClient(Node):
    def __init__(self):
        super().__init__('waypoint_client')
        self.pose = None
        self.busy = False
        self.finished = False
        self.client = self.create_client(A439129, 'get_waypoint_error')
        self.pose_sub = self.create_subscription(
            Pose, '/turtle1/pose', self.on_pose, 10)
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.step)

    def on_pose(self, pose):
        self.pose = pose

    def step(self):
        if (self.finished or self.busy or self.pose is None
                or not self.client.service_is_ready()):
            return
        self.cmd_pub.publish(Twist())
        req = A439129.Request()
        req.x_current = float(self.pose.x)
        req.y_current = float(self.pose.y)
        self.busy = True
        future = self.client.call_async(req)
        future.add_done_callback(self.on_response)

    def on_response(self, future):
        self.busy = False
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f'Service call failed: {exc}')
            return
        if result.complete:
            self.cmd_pub.publish(Twist())
            self.get_logger().info('All waypoints complete.')
            self.finished = True
            return
        if result.success:
            self.cmd_pub.publish(Twist())
            return
        heading_error = normalize_angle(result.angle - self.pose.theta)
        cmd = Twist()
        cmd.angular.z = max(-2.0, min(2.0, 4.0 * heading_error))
        if abs(heading_error) < 0.15:
            cmd.linear.x = min(1.5 * result.distance, 1.5)
        self.cmd_pub.publish(cmd)


def service_server_main(args=None):
    rclpy.init(args=args)
    node = WaypointService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def service_client_main(args=None):
    rclpy.init(args=args)
    node = WaypointClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
