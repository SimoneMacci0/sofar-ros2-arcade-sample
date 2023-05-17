from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

import rclpy
import time

from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Bool, Int64
from geometry_msgs.msg import Point

from ros2_arcade_sample_interface2.action import WaypointsAction


class WaypointsServer(Node):

    def __init__(self):
        super().__init__("waypoints_action_server")

        self.idle = True

        self.ack_sub = self.create_subscription(
            Bool,
            "/ack", 
            self.on_ack, 
            10,
        )
        self.waypoints_server = ActionServer(
           self,
            WaypointsAction,
            "/waypoints",
            self.waypoints_callback,
        )
        self.waypoint_pub = self.create_publisher(Point, "/goal", 10)


    def on_ack(self, msg: Bool):
        self.idle = msg.data


    def waypoints_callback(self, goal_handle: ServerGoalHandle):
        feedback_msg = WaypointsAction.Feedback()
        n = len(goal_handle.request.waypoints)

        for i, waypoint in enumerate(goal_handle.request.waypoints):

            feedback_msg.remaining.data = n - i
            goal_handle.publish_feedback(feedback_msg)

            next_goal = Point()
            next_goal.x = waypoint.x
            next_goal.y = waypoint.y
            self.waypoint_pub.publish(next_goal)
            self.idle = False

            while not self.idle:
                time.sleep(1)

        goal_handle.succeed()

        result = WaypointsAction.Result()
        result.done.data = True
        return result




    

        
def main():
    rclpy.init(args = None)

    server = WaypointsServer()
    executor = MultiThreadedExecutor()
    executor.add_node(server)

    try:
        executor.spin()
    except KeyboardInterrupt:
        executor.shutdown()
        server.destroy_node()
        rclpy.shutdown()
    


if __name__ == '__main__':
    main()
