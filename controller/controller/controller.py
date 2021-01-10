"""
Controller node
""" 


import rclpy
from rclpy.node import Node

import time

from control_messages.msg import Pose
from control_messages.msg import Control
from std_msgs.msg import Header

from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy

# Define the ROS2 QOS used by this node.
BEST_EFFORT_QOS_PROFILE = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE)

class Controller(Node):
    """ Node to control an ackermann steer vehicle a simple vehicle model."""

    def __init__(self):
        """ Create a ROS2 node and ROS2 parts """
        # Call parent initialisers
        Node.__init__(self, "controller")

        self.pose_msg = Pose()
        self.control_msg = Control()

        # Create pose subscriber
        self.create_subscription(
            msg_type=Pose,
            topic="/pose",
            callback=self.receive_pose_msg,
            qos_profile=BEST_EFFORT_QOS_PROFILE
            )

        # Create control publisher
        self.control_publisher = self.create_publisher(
            msg_type=Control,
            topic="/control",
            qos_profile=BEST_EFFORT_QOS_PROFILE
            )


        # Setup timers to spin the execution loop. 
        self.create_timer(1.0/30.0, self.execute)

    def execute(self):

        
        self.control_msg.steering = 1.0

        if self.pose_msg.velocity_x < 10.0:
            self.control_msg.acceleration = 0.1
        else:
            self.control_msg.acceleration = 0.0


        # publish control
        self.control_publisher.publish(self.control_msg)


    def receive_pose_msg(self, pose_msg):
        self.pose_msg = pose_msg


def main(args=None):
    """Default entry point for the ROS2 node.

    Args:
        args (tuple, optional): Args to pass to rclpy init(). Defaults to None.
    """
    rclpy.init(args=args)

    controller = Controller()

    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()

    # Try to shutdown. We should have already shutdown when the user exited the UI.
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()

