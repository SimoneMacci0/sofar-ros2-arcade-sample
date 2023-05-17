import arcade
from threading import Thread
import math
from ament_index_python.packages import get_package_share_directory

SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600
SCREEN_TITLE = "ROS2-Arcade Sample"

# Thread to run simulation in background
class SimThread(Thread):
   def __init__(self):
      Thread.__init__(self)
   
   def run(self):
      arcade.run()

# Main class representing simulation environment
class ROS2ArcadeSim(arcade.Window):

    def __init__(self):
        super().__init__(SCREEN_WIDTH, SCREEN_HEIGHT, SCREEN_TITLE)
        arcade.set_background_color(arcade.color.WHITE)

        # Start game loop in background
        self.thread = SimThread()
    
        # Robot
        self.robot = arcade.Sprite(":resources:images/topdown_tanks/tankBody_red_outline.png", scale=0.8)

        # Utilities
        self.is_drawing = False
        self.points_list = None
        self.base_share_directory = get_package_share_directory("ros2_arcade_sample")

    # Setup method invoked at game start
    def setup(self):
        self.robot.center_x = 100.0
        self.robot.center_y = 100.0
        self.points_list = arcade.SpriteList(use_spatial_hash=True)

    # Draw robot on map every frame
    def on_draw(self):
        arcade.start_render()
        self.points_list.draw()
        self.robot.draw()
        
    # Retrieve robot's pose for publication
    def get_robot_pose(self):
        return self.robot.center_x, self.robot.center_y, (self.robot.angle + 90)

    # Method to update robot's pose from control twist, using robot's kinematics
    def update_robot_pose(self, speed: float, ang_speed: float):
        # Convert angle in degrees to radians.
        angle_rad = math.radians(self.robot.angle)
        # Rotate robot
        self.robot.angle += ang_speed
        
        # Use math to find position change based on speed and angle
        self.robot.center_x += -speed * math.sin(angle_rad)
        self.robot.center_y += speed * math.cos(angle_rad)

        if self.is_drawing:
            new_waypoint = arcade.Sprite(self.base_share_directory + "/resource/point.png", 0.2)
            new_waypoint.center_x = self.robot.center_x
            new_waypoint.center_y = self.robot.center_y
            self.points_list.append(new_waypoint)