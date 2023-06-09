from rclpy.node import Node
import rclpy
import math
import time

from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Pose2D, Point

class PositionController(Node):

    def __init__(self):
        super().__init__("controller_node")

        # Variables
        self.pose = None
        self.threshold = 5.0

        # Utilities
        self.starting_pose = None
        self.got_first_pose = False
        self.counter = 0

        # Control gains
        self.Kl = 0.015
        self.Ka = 4.0

        # subscribers
        self.goal_sub = self.create_subscription(Point, "/goal", self.on_new_goal_received, 10)
        self.pose_sub = self.create_subscription(Pose2D, "/robot_pose", self.on_pose_received, 10)

        # Publishers
        self.ack_pub = self.create_publisher(Bool, "/ack", 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

    # Callback for setting new goal for conttroller
    def on_new_goal_received(self, msg: Point):
        self.set_new_target(msg.x, msg.y)

    # Pose callback
    def on_pose_received(self, msg: Pose2D):
        self.pose = msg
        # Store initial location for future usage
        if not self.got_first_pose:
            self.starting_pose = msg
            self.got_first_pose = True

    # Set new goal for controller
    def set_new_target(self, x: float, y: float):

        self.goal_x = x
        self.goal_y = y
        # Start control timer
        self.control_loop_timer = self.create_timer(0.02, self.on_control_loop)

    # Control loop callback
    def on_control_loop(self):
        # compute distance error
        e_d = math.sqrt((self.pose.x - self.goal_x)**2 + (self.pose.y - self.goal_y)**2)
        # compute angular error
        e_a = math.atan2(self.goal_y - self.pose.y, self.goal_x - self.pose.x) - self.pose.theta
        # Clip rotation so it's in [-pi, pi]
        if e_a > math.pi:
            e_a -= 2*math.pi
        elif e_a < -math.pi:
            e_a += 2*math.pi 

        # exit loop if goal reached
        if e_d <= self.threshold:
            # cancel timer
            self.control_loop_timer.cancel()
            # Publish ack msg
            ack = Bool()
            ack.data = True
            self.ack_pub.publish(ack)
            return
           
        # Compute control inputs
        lin_control = self.Kl * e_d 
        ang_control = self.Ka * e_a

        # Build twist msg
        cmd_vel = Twist()
        cmd_vel.linear.x = lin_control
        cmd_vel.angular.z = ang_control

        # Publish it
        self.cmd_vel_pub.publish(cmd_vel)


def main():
    rclpy.init(args = None)

    controller = PositionController()
    #time.sleep(10)
    #controller.set_new_target()

    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
