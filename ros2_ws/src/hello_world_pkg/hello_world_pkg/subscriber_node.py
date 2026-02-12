#!/usr/bin/env python3
"""
ROS2 Subscriber Node - Hello World

This node subscribes to /hello_topic and prints received messages.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class HelloWorldSubscriber(Node):
    """A simple subscriber node that listens to hello world messages."""

    def __init__(self):
        """Initialize the subscriber node."""
        super().__init__('hello_world_subscriber')
        
        # Create subscription to /hello_topic with queue size of 10
        self.subscription = self.create_subscription(
            String,
            'hello_topic',
            self.listener_callback,
            10
        )
        
        # Prevent unused variable warning
        self.subscription
        
        self.get_logger().info('Hello World Subscriber has been started')

    def listener_callback(self, msg):
        """Callback function for received messages."""
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    """Main function to initialize and run the subscriber node."""
    # Initialize ROS2 Python client library
    rclpy.init(args=args)
    
    # Create the node
    node = HelloWorldSubscriber()
    
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
