# Install ROS2 Dependencies
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import copy

class LiftROS2Handler(Node):

    def __init__(self):
        super().__init__("lift_ros2_handler")

        # Initiate publisher
        self.publisher_ = self.create_publisher(String, "robot_exit_status", 5)


    # Returns all items in queue
    def get_request_data(self) -> list:
        if len(self.request_data_queue) == 0: #  If queue length is 0, do not update this queue
            return None
        else:
            temp = copy.deepcopy(self.request_data_queue)
            self.request_data_queue = []
            return temp

    def publish_robot_exit_status_to_robot(self, robot_exit_status):
        # Handle logic of what lift state is on the other side, not here
        # here, just take it as the door is open and we gotta broadcast to fleet manager to ask it to move the robot
        msg = String()
        msg.data = robot_exit_status
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)




def main(args=None):
    rclpy.init(args=args)

    fleet_ros2_handler = FleetROS2Handler()

    rclpy.spin(fleet_ros2_handler)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    fleet_ros2_handler.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
