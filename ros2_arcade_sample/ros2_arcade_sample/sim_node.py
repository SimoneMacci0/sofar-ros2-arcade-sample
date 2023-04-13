
import arcade
import rclpy
from rclpy.node import Node
# Import arcade simulation environment from library script
from .lib.simulation import ROS2ArcadeSim

from geometry_msgs.msg import Twist


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


    # Callback called on receiving velocity commands
    def twist_callback(self, msg: Twist):
        # Invoke internal simulation method updating robot's position with twist msg content
        self.sim.update_robot_pose(msg.linear.x, msg.angular.z)


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