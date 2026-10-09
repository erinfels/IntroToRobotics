"""csci3302_lab3 controller."""

#Set variable to 1 in order to test proportial controller
#Set variable to 2 in order to test turn_drive_turn

whichcontroller = 1



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
AXLE_LENGTH = EPUCK_AXLE_DIAMETER = 0.053 # ePuck's wheels are 53mm apart.
EPUCK_MAX_WHEEL_SPEED = 0.1257 # ePuck wheel speed in m/s
MAX_SPEED = 6.28
WHEEL_RADIUS = EPUCK_MAX_WHEEL_SPEED / MAX_SPEED
DIST_TOL = 0.05
ANGLE_TOL = 0.05

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

#Phi_l = X_R/r - d*theta/2r
#Phi_r = X_R/r + d*theta/2r

#Determine (Position Error) Calculate the Euclidean distance 𝜌 between your current location and the goal position.
PosErr = 0

#Determine (Bearing Error) Calculate the angle 𝛼 between the orientation of the robot and the direction of the goal position. (positive to the left)
BerErr = 0

#Determine (Heading Error) Calculate the angle 𝜂 between the orientation of the robot and the goal orientation.
HedErr = 0

def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))
    
    
MAX_TURN_WHEEL = 3.0

def wheel_speeds(x_dot, theta_dot):
    turn = theta_dot * AXLE_LENGTH / 2 / WHEEL_RADIUS
    turn = max(-MAX_TURN_WHEEL, min(MAX_TURN_WHEEL, turn))
    
    headroom = MAX_SPEED - abs(turn)
    fwd = max(-headroom, min(headroom, x_dot / WHEEL_RADIUS))
    
    phi_l = max(-MAX_SPEED, min(MAX_SPEED, fwd - turn))
    phi_r = max(-MAX_SPEED, min(MAX_SPEED, fwd + turn))
    return phi_l, phi_r


state = "turn_to_goal"

def turn_drive_turn_control(rho, alpha, eta):
    global state
    x_dot, theta_dot, done = 0.0, 0.0, False
    if state == "turn_to_goal":
        theta_dot = 2.0 * alpha
        if abs(alpha) < ANGLE_TOL:
            state = "drive"
    elif state == "drive":
        x_dot = EPUCK_MAX_WHEEL_SPEED
        theta_dot = 2.0 * alpha
        if rho < DIST_TOL:
            state = "turn_to_heading"
    elif state == "turn_to_heading":
        theta_dot = 2.0 * eta
        if abs(eta) < ANGLE_TOL:
            state = "turn_to_goal"
            done = True
    return x_dot, theta_dot, done

P1, P2, P3 = 1.0, 4.0, 0.2  
TURN_GAIN = 2.0              

def proportional_controller(rho, alpha, eta):
    if rho > DIST_TOL:
        x_dot = P1 * rho * max(0.0, math.cos(alpha))
        theta_dot = P2 * alpha + P3 * eta
        return x_dot, theta_dot, False
    return 0.0, TURN_GAIN * eta, abs(eta) < ANGLE_TOL
    

# Main Control Loop:
while robot.step(SIM_TIMESTEP) != -1:
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
    
    gx, gy = waypoints[index]
    prev_x, prev_y = waypoints[index - 1]   # index 0 wraps to the last waypoint automatically
    BerErr = wrap(math.atan2(gy - pose_y, gx - pose_x) - pose_theta)
    HedErr = wrap(math.atan2(gy - prev_y, gx - prev_x) - pose_theta)
 
    if (whichcontroller == 1):
        x_dot, theta_dot, done = proportional_controller(PosErr, BerErr, HedErr)  
    else:
        x_dot, theta_dot, done = turn_drive_turn_control(PosErr, BerErr, HedErr)
    vL, vR = wheel_speeds(x_dot, theta_dot)
    
    if done:
        index = (index + 1) % len(waypoints)
    
    print(f"{PosErr} {index}")
    print(f"{BerErr} {HedErr}")
    print("Current pose: [%5f, %5f, %5f]" % (pose_x, pose_y, pose_theta))
    leftMotor.setVelocity(vL)
    rightMotor.setVelocity(vR)
