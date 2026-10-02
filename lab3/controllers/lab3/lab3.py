"""csci3302_lab3 controller."""

# You may need to import some classes of the controller module.
import math
from controller import Robot, Motor, DistanceSensor, Supervisor
import numpy as np #Specifically np.atan2 and np.sqrt

pose_x = 0
pose_y = 0
pose_theta = 0

# create the Robot instance.
robot = Supervisor()

# ePuck Constants
EPUCK_AXLE_DIAMETER = 0.053 # ePuck's wheels are 53mm apart.
EPUCK_MAX_WHEEL_SPEED = 0.1257 # ePuck wheel speed in m/s
MAX_SPEED = 6.28

# get the time step of the current world.
SIM_TIMESTEP = int(robot.getBasicTimeStep())

# Initialize Motors
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# Initialize and Enable the Ground Sensors
gsr = [0, 0, 0]
ground_sensors = [robot.getDevice('gs0'), robot.getDevice('gs1'), robot.getDevice('gs2')]
for gs in ground_sensors:
    gs.enable(SIM_TIMESTEP)

# Allow sensors to properly initialize
for i in range(10): robot.step(SIM_TIMESTEP)  

vL = 0
vR = 0

# Initialize gps and compass for odometry
gps = robot.getDevice("gps")
gps.enable(SIM_TIMESTEP)
compass = robot.getDevice("compass")
compass.enable(SIM_TIMESTEP)

# TODO: Find waypoints to navigate around the arena while avoiding obstacles
# Use shift+drag on the ping pong marker in the simulator to find good waypoints.
# Add them as (x, y) tuples. You need at least one waypoint before running!
waypoints = [(-0.314705, -0.084838), (-0.314705, -0.414838),(0.325295, -0.414838),(0.325295, -0.254838),(0.045295, -0.014838),(0.345295, 0.265162),(0.135295, 0.415162),(-0.304705, 0.415162),(-0.304705, 0.295162),(-0.184705, 0.285162),(-0.184705, -0.004838)]

# Index indicating which waypoint the robot is reaching next
index = 1

# Get ping pong ball marker that marks the next waypoint the robot is reaching
marker = robot.getFromDef("marker").getField("translation")

#Phi_l = X_R/r - dtheta/2r
#Phi_r = X_R/r + dtheta/2r

#Determine (Position Error) Calculate the Euclidean distance 𝜌 between your current location and the goal position.
PosErr = 0

#Determine (Bearing Error) Calculate the angle 𝛼 between the orientation of the robot and the direction of the goal position. (positive to the left)
BerErr = 0


#Determine (Heading Error) Calculate the angle 𝜂 between the orientation of the robot and the goal orientation.
HedErr = 0

#theres a decent chance these shouldn't be functions lol

def turn_drive_turn_control(): 
    #Using <left/right>motor.setVelocity(), create a controller that rotates in place until the robot is facing the
    #goal position (reduce bearing error), drives forward to the goal position (reduce position error), then
    #rotates in place to orient to the proper heading (reduce heading error).
    return none

def proportional_controller():
    #Calculate the necessary change in robot position 𝑋̇𝑅 that is
    #proportional to 𝜌. Calculate the necessary change in robot rotation 𝜃̇𝑅 that is proportional to 𝛼 and 𝜂.
    #Set values for left and right wheel motors accordingly.
    
    #Create a proportional feedback controller that uses the inverse kinematics equations with your error
    #signals to compute the wheel rotations needed to make the position and rotation changes for driving to
    #a given goal.
    return none
    

# Main Control Loop:
while robot.step(SIM_TIMESTEP) != -1:
    # Safety check: make sure waypoints are defined
    if len(waypoints) == 0:
        print("ERROR: No waypoints defined! Please add waypoints to the waypoints list.")
        leftMotor.setVelocity(0.0)
        rightMotor.setVelocity(0.0)
        continue

    # Set the position of the marker
    marker.setSFVec3f([waypoints[index][0], waypoints[index][1], 0.01])
    
    # Read ground sensor values
    for i, gs in enumerate(ground_sensors):
        gsr[i] = gs.getValue()

    # Read pose_x, pose_y, pose_theta from gps and compass
    pose_x = gps.getValues()[0]
    pose_y = gps.getValues()[1]
    pose_theta = np.arctan2(compass.getValues()[0], compass.getValues()[1])
    
    
    PosErr = np.sqrt((pose_x-waypoints[index][0])**2 + (pose_y-waypoints[index][1])**2)
    BerErr = pose_theta
    
    # TODO: controller
    
    print(f"{PosErr} {index}")
    print(f"{BerErr} {HedErr}")
    print("Current pose: [%5f, %5f, %5f]" % (pose_x, pose_y, pose_theta))
    leftMotor.setVelocity(vL)
    rightMotor.setVelocity(vR)
