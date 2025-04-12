#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from std_msgs.msg import Bool

class NavigateNode(Node):
    def __init__(self):
        super().__init__('navigate_node')

        # get target position
        self.declare_parameter('target_x', 3.0)
        self.declare_parameter('target_y', 3.0)

        self.target_x = self.get_parameter('target_x').value
        self.target_y = self.get_parameter('target_y').value

        # create navigator
        self.navigator = BasicNavigator()
        self.navigation_started = False

        # check aliment and navigation status
        self.create_subscription(Bool, '/initial_alignment_done', self.alignment_done, 10)
        
    # check if initial alignment is done
    def alignment_done(self, msg):
        if msg.data:
            self.start_navigation()

    # navigate
    def start_navigation(self):
        if self.navigation_started:
            return
        
        self.navigator.wait_until_nav2_active()

        goal = PoseStamped()
        goal.header.frame_id = "map"  
        goal.header.stamp = self.get_clock().now().to_msg()

        goal.pose.position.x = self.target_x
        goal.pose.position.y = self.target_y 

        self.navigator.go_to_pose(goal)
        
        self.navigation_started = True

        # check navigation status
        while not self.navigator.is_nav_complete():
            rclpy.spin_once(self)

        result = self.navigator.get_result()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("Navigation completed successfully.")
        else:
            self.get_logger().error("Navigation failed.")
        
def main(args=None):
    rclpy.init(args=args)
    node = NavigateNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
