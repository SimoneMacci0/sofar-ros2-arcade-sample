
import arcade
import rclpy
import math

from rclpy.node import Node

# Import arcade simulation environment from library script
from .lib.simulation import ROS2ArcadeSim

from geometry_msgs.msg import Twist, Pose2D

from ros2_arcade_sample_interface2.srv import Draw


# ROS2 node class wrapping arcade simulation for external control
class ROS2ArcadeSimNode(Node):

    def __init__(self):
        super().__init__("ros2_arcade_sim_node")

        # Instantiate simulation class
        self.sim = ROS2ArcadeSim()
        # Initialize simulation
        self.sim.setup()
        # Start simulation loop in background thread
        self.sim.thread.start()

        # Subscriber for twist command
        self.twist_sub = self.create_subscription(Twist, "/cmd_vel", self.twist_callback, 10)

        # Publisher for robot pose
        self.pose_pub = self.create_publisher(Pose2D, "/robot_pose", 10)

        # Timer for robot pose publication
        self.pose_timer = self.create_timer(0.01, self.timer_callback)

        self.drawing_srv = self.create_service(Draw, "/draw", self.drawing_callback)

    def drawing_callback(self, request: Draw.Request, response: Draw.Response):
        self.sim.is_drawing = request.mode.data
        response.status.data = self.sim.is_drawing
        return response

    # Callback called on receiving velocity commands
    def twist_callback(self, msg: Twist):
        # Invoke internal simulation method updating robot's position with twist msg content
        self.sim.update_robot_pose(msg.linear.x, msg.angular.z)


    # Callback invoked whenever timer elapses
    def timer_callback(self):
        x, y, theta = self.sim.get_robot_pose()
        pose_msg = Pose2D()
        pose_msg.x = x
        pose_msg.y = y
        pose_msg.theta = math.radians(theta)
        self.pose_pub.publish(pose_msg)

    
def main(args=None):
    
    # Instantiate node 
    rclpy.init(args=args)
    simulation_node = ROS2ArcadeSimNode()
    # Spin indefinitely
    rclpy.spin(simulation_node)
    # On shutdown...
    simulation_node.destroy_node()
    rclpy.shutdown()
    

# Script entry point
if __name__ == "__main__":
    main()