#!/usr/bin/python3
import rclpy
from rclpy.node import Node

from example_interfaces.msg import String


class FlightController(Node):
    def __init__(self):
        super().__init__("flight_controller_node")        
        self.number_subscriber_ = self.create_subscription(
            String, "news", self.callback_news, 10)
        self.get_logger().info("msg subscriber has been started.")

    def callback_news(self, msg):
        self.get_logger().info(msg.data)
        # self.counter_ += msg.data
        # new_msg = String()
        # new_msg.data = self.counter_
        # self.number_count_publisher_.publish(new_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FlightController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

