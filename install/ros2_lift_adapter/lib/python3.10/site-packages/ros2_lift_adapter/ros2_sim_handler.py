import tkinter as tk
from functools import partial

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from multiprocessing import Process, Pipe


class ROS2SimPublisher(Node):

    def __init__(self):
        super().__init__('ros2_sim_publisher')
        self.publisher_ = self.create_publisher(String, 'fleet_adapter/requests', 10)

    def publish_lift_request(self, data):
        msg = String()  
        msg.data = data
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    print("Initialised ROS2 Lift Request Simulator")
    rclpy.init(args=args)   

    ros2_sim_publisher = ROS2SimPublisher()

    rclpy.spin(ros2_sim_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ros2_sim_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()