#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool

class InitialAlignmentNode(Node):
    def __init__(self):
        super().__init__('initial_alignment_node')
        
        # get initial position
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)

        self.initial_x = self.get_parameter('initial_x').value
        self.initial_y = self.get_parameter('initial_y').value
    
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.done_publisher = self.create_publisher(Bool, '/initial_alignment_done', 10)

        self.done = False

    # publish initial position
    def publish_initial_pose(self):
        if self.done:
            return

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = self.initial_x
        msg.pose.pose.position.y = self.initial_y

        msg.pose.covariance[0] = 0.05
        msg.pose.covariance[7] = 0.05
        msg.pose.covariance[35] = 0.05

        self.publisher.publish(msg)
        
        # tell navigation node when done
        done_msg = Bool()
        done_msg.data = True
        self.done_publisher.publish(done_msg)

        self.done = True

def main(args=None):
    rclpy.init(args=args)
    node = InitialAlignmentNode()
    node.publish_initial_pose()
    rclpy.shutdown()

if __name__ == '__main__':
    main()