#!/usr/bin/python3
import rclpy
from rclpy.node import Node

from example_interfaces.msg import String


class CameraNode(Node):
    def __init__(self):
    	#publishing number as an example of publisher node 
        super().__init__("camera_node")
        #self.number_ = 2
        self.camera_node_publisher_ = self.create_publisher(String, "news", 10)
        #publishes with this rate
        self.number_timer_ = self.create_timer(0.5, self.publish_news)
        self.get_logger().info("msg publisher has been started.")

    def publish_news(self):
        msg = String()
        msg.data = "hello"
        self.camera_node_publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

