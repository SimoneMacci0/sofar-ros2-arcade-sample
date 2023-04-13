import arcade
from threading import Thread
import math

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

    # Setup method invoked at game start
    def setup(self):
        self.robot.center_x = SCREEN_WIDTH / 2
        self.robot.center_y = SCREEN_HEIGHT / 2

    # Draw robot on map every frame
    def on_draw(self):
        self.clear()
        self.robot.draw()

    # Method to update robot's pose from control twist, using robot's kinematics
    def update_robot_pose(self, speed: float, ang_speed: float):
        # Convert angle in degrees to radians.
        angle_rad = math.radians(self.robot.angle)
        # Rotate robot
        self.robot.angle += ang_speed
        # Use math to find position change based on speed and angle
        self.robot.center_x += -speed * math.sin(angle_rad)
        self.robot.center_y += speed * math.cos(angle_rad)