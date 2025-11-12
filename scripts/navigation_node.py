#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_srvs.srv import Trigger

class NavigationNode(Node):

    def __init__(self):
        super().__init__('navigation_node')

        # Publisher para pose inicial (AMCL)
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10)

        # Action client para enviar goals
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Serviço que envia o objetivo
        self.create_service(Trigger, '/start_navigation', self.send_goal_callback)

        # Publicar pose inicial com pequeno atraso
        self.timer = self.create_timer(2.0, self.publish_initial_pose_once)
        self.initial_pose_published = False

        self.get_logger().info('Service /start_navigation ready')

    def publish_initial_pose_once(self):
        if self.initial_pose_published:
            return

        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = 'map'
        pose.pose.pose.position.x = 0.0
        pose.pose.pose.position.y = 0.0
        pose.pose.pose.orientation.z = 1.0
        pose.pose.pose.orientation.w = 0.0

        self.initial_pose_pub.publish(pose)
        self.initial_pose_published = True
        self.get_logger().info('Initial pose published to /initialpose')

    def send_goal_callback(self, request, response):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = 4.0
        goal_msg.pose.pose.position.y = 0.2
        goal_msg.pose.pose.orientation.w = 1.0

        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('NavigateToPose action server not available')
            response.success = False
            response.message = 'Nav2 server unavailable'
            return response

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self.get_logger().info('Navigation goal sent')
        response.success = True
        response.message = 'Goal sent to Nav2'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
