# Install ROS2 Dependencies
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import copy

class FleetROS2Handler(Node):

    def __init__(self):
        super().__init__("fleet_ros2_handler")

        # Initiate publisher
        self.publisher_ = self.create_publisher(String, "lift_state", 5)

        # Initiate subscriber to requests
        self.subscription = self.create_subscription(
            String,
            'fleet_adapter/requests', # Denotes that the lift request comes from the first layer (flt mgr)
            self.append_request_data_to_queue,
            10)

        self.request_data_queue = []

    def append_request_data_to_queue(self, msg):
        request = msg.data

        # Assume the data comes in the following format:
        # request_level;destination_level
        request_level, destination_level = request.split(";")
        request_data = {
            "request_level": request_level,
            "destination_level": destination_level
        }

        self.request_data_queue.append(request_data)

    # Returns all items in queue
    def publish_lift_state_to_fleet_manager(self, liftState):
        # Handle logic of what lift state is on the other side, not here
        # here, just take it as the door is open and we gotta broadcast to fleet manager to ask it to move the robot
        msg = String()
        msg.data = liftState
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

def check_for_subscriptions(node):
    # Spin the ndoe once
    rclpy.spin_once(node, timeout_sec=1.0)
    if len(node.request_data_queue) == 0: #  If queue length is 0, do not update this queue
        return None
    else:
        temp = copy.deepcopy(node.request_data_queue)
        node.request_data_queue = []
        return temp

if __name__ == '__main__':
    main()
