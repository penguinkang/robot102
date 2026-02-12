#!/usr/bin/env python3
"""
ROS2 Publisher Node - Hello World

This node publishes "Hello World" messages to the /hello_topic at 1 Hz.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class HelloWorldPublisher(Node):
    """A simple publisher node that sends hello world messages."""

    def __init__(self):
        """Initialize the publisher node."""
        super().__init__('hello_world_publisher')
        
        # Create publisher on /hello_topic with queue size of 10
        self.publisher_ = self.create_publisher(String, 'hello_topic', 10)
        
        # Create timer that calls timer_callback every 1 second
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Message counter
        self.counter = 0
        
        self.get_logger().info('Hello World Publisher has been started')

    def timer_callback(self):
        """Publish a hello world message."""
        msg = String()
        msg.data = f'Hello World: {self.counter}'
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        
        self.counter += 1


def main(args=None):
    """Main function to initialize and run the publisher node."""
    # Initialize ROS2 Python client library
    rclpy.init(args=args)
    
    # Create the node
    node = HelloWorldPublisher()
    
    try:
        # Keep the node running
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
