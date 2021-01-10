"""
Vehicle Sim node
"""


import rclpy
from rclpy.node import Node

import time
import math

from control_messages.msg import Pose
from control_messages.msg import Control
from std_msgs.msg import Header
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from vehicle_sim.model import Model

from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy

# Define the ROS2 QOS used by this node.
BEST_EFFORT_QOS_PROFILE = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE)

class VehicleSim(Node):
    """ Node to simulate a simple vehicle model."""

    def __init__(self):
        """ Create a ROS2 node and ROS2 parts """
        # Call parent initialisers
        # SecmUtilityCore.__init__(self)
        Node.__init__(self, "vehicle_sim")
        # super().__init__('vehicle_sim')

        self.vehicle_marker_array = MarkerArray()
        self.vehicle_marker = Marker()
        self.pose_msg = Pose()
        self.control_msg = Control()

        self.model = Model()

        # Create subscribers to listen to SECM output
        self.create_subscription(
            msg_type=Control,
            topic="/control",
            callback=self.receive_control_msg,
            qos_profile=BEST_EFFORT_QOS_PROFILE
            )

        # Create pose publisher
        self.pose_publisher = self.create_publisher(
            msg_type=Pose,
            topic="/pose",
            qos_profile=BEST_EFFORT_QOS_PROFILE
            )

        # Create marker publisher
        self.vehicle_marker_publisher = self.create_publisher(
            msg_type=Marker,
            topic="/vehicle_marker",
            qos_profile=BEST_EFFORT_QOS_PROFILE
            )

        # Setup timers to spin the execution loop. 
        self.create_timer(1.0/30.0, self.execute)

    def execute(self):

        self.model.control(self.control_msg.acceleration, self.control_msg.brake, self.control_msg.steering)

        self.model.update(1.0/30.0)

        # update the model

        # pack the pose
        self.pose_msg.easting = self.model.easting
        self.pose_msg.northing = self.model.northing
        self.pose_msg.yaw = self.model.yaw
        self.pose_msg.velocity_x = self.model.velocity_x
        self.pose_msg.velocity_y = self.model.velocity_y


        # pack the marker array
        self.vehicle_marker.header.frame_id = "world"
        self.vehicle_marker.header.stamp.sec = int(time.time())
        self.vehicle_marker.header.stamp.nanosec = int(time.time()*1e9 % 1e9)
        self.vehicle_marker.ns = "my_namespace"
        self.vehicle_marker.id = 0

        self.vehicle_marker.type = self.vehicle_marker.ARROW
        self.vehicle_marker.action = self.vehicle_marker.ADD
        self.vehicle_marker.scale.x = 3.0
        self.vehicle_marker.scale.y = 1.0
        self.vehicle_marker.scale.z = 1.0
        self.vehicle_marker.color.a = 1.0 # Don't forget to set the alpha!
        self.vehicle_marker.color.r = 0.0
        self.vehicle_marker.color.g = 1.0
        self.vehicle_marker.color.b = 0.0
        self.vehicle_marker.pose.position.x = self.model.easting
        self.vehicle_marker.pose.position.y = self.model.northing
        self.vehicle_marker.pose.position.z = 0.0

        quarternion = self.to_quarternion(0.0, 0.0, self.pose_msg.yaw)

        self.vehicle_marker.pose.orientation.x = quarternion[0]
        self.vehicle_marker.pose.orientation.y = quarternion[1]
        self.vehicle_marker.pose.orientation.z = quarternion[2]
        self.vehicle_marker.pose.orientation.w = quarternion[3]

        # publish pose
        self.pose_publisher.publish(self.pose_msg)

        # publish vehicle marker
        self.vehicle_marker_publisher.publish(self.vehicle_marker)


    def receive_control_msg(self, control_msg):
        self.control_msg = control_msg

    def to_quarternion(self, roll, pitch, yaw):
        #  Abbreviations for the various angular functions
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        # Quaternion q
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        return [x, y, z, w]


def main(args=None):
    """Default entry point for the ROS2 node.

    Args:
        args (tuple, optional): Args to pass to rclpy init(). Defaults to None.
    """
    rclpy.init(args=args)

    vehicle_sim = VehicleSim()

    rclpy.spin(vehicle_sim)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    vehicle_sim.destroy_node()

    # Try to shutdown. We should have already shutdown when the user exited the UI.
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()

