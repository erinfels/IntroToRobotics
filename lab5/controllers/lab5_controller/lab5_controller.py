"""lab5 controller."""
from controller import Robot, Motor, Camera, RangeFinder, Lidar, Keyboard
import math
import numpy as np
from matplotlib import pyplot as pyplot
 
MAX_SPEED = 7.0  # [rad/s]
MAX_SPEED_MS = 0.633 # [m/s]
AXLE_LENGTH = 0.4044 # m
MOTOR_LEFT = 10
MOTOR_RIGHT = 11
N_PARTS = 12
 
LIDAR_ANGLE_BINS = 667
LIDAR_SENSOR_MAX_RANGE = 2.75 # Meters
LIDAR_ANGLE_RANGE = math.radians(240)
 
 
##### vvv [Begin] Do Not Modify vvv #####
 
# create the Robot instance.
robot = Robot()
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
 
# The Tiago robot has multiple motors, each identified by their names below
part_names = ("head_2_joint", "head_1_joint", "torso_lift_joint", "arm_1_joint",
              "arm_2_joint",  "arm_3_joint",  "arm_4_joint",      "arm_5_joint",
              "arm_6_joint",  "arm_7_joint",  "wheel_left_joint", "wheel_right_joint")
 
# All motors except the wheels are controlled by position control. The wheels
# are controlled by a velocity controller. We therefore set their position to infinite.
target_pos = (0.0, 0.0, 0.09, 0.07, 1.02, -3.16, 1.27, 1.32, 0.0, 1.41, 'inf', 'inf')
robot_parts=[]
 
for i in range(N_PARTS):
    robot_parts.append(robot.getDevice(part_names[i]))
    robot_parts[i].setPosition(float(target_pos[i]))
    robot_parts[i].setVelocity(robot_parts[i].getMaxVelocity() / 2.0)
 
# The Tiago robot has a couple more sensors than the e-Puck
# Some of them are mentioned below. We will use its LiDAR for Lab 5
 
range = robot.getDevice('range-finder')
range.enable(timestep)
camera = robot.getDevice('camera')
camera.enable(timestep)
camera.recognitionEnable(timestep)
lidar = robot.getDevice('Hokuyo URG-04LX-UG01')
lidar.enable(timestep)
lidar.enablePointCloud()
 
# We are using a GPS and compass to disentangle mapping and localization
gps = robot.getDevice("gps")
gps.enable(timestep)
compass = robot.getDevice("compass")
compass.enable(timestep)
 
# We are using a keyboard to remote control the robot
keyboard = robot.getKeyboard()
keyboard.enable(timestep)
 
# The display is used to display the map. We are using 360x360 pixels to
# map the 12x12m2 apartment
display = robot.getDevice("display")
 
# Odometry
pose_x     = 0
pose_y     = 0
pose_theta = 0
 
vL = 0
vR = 0
 
lidar_sensor_readings = [] # List to hold sensor readings
lidar_offsets = np.linspace(-LIDAR_ANGLE_RANGE/2., +LIDAR_ANGLE_RANGE/2., LIDAR_ANGLE_BINS)
lidar_offsets = lidar_offsets[83:len(lidar_offsets)-83] # Only keep lidar readings not blocked by robot chassis
 
# map = None
##### ^^^ [End] Do Not Modify ^^^ #####
 
##################### IMPORTANT #####################
# Set the mode here. Please change to 'autonomous' before submission
mode = 'manual' # Part 1.1: manual mode
# mode = 'autonomous'
 
######################
#
# Map Initialization
#
######################
 
# Part 1.2: Map Initialization
 
# Initialize your map data structure here as a 2D floating point array
map = np.zeros(shape=[360,360])
waypoints = []
 
MAP_INCREMENT = 5e-3   # evidence added per LIDAR hit (~200 hits to reach 1.0)
MAP_THRESHOLD = 0.5    # cells below this are treated as noise when saving
 
while robot.step(timestep) != -1 and mode != 'planner':
 
    ###################
    #
    # Mapping
    #
    ###################
 
    ################ v [Begin] Do not modify v ##################
    # Ground truth pose
    pose_x = gps.getValues()[0]
    pose_y = gps.getValues()[1]
    
    n = compass.getValues()
    rad = -((math.atan2(n[0], n[2]))-1.5708)
    pose_theta = rad
 
    lidar_sensor_readings = lidar.getRangeImage()
    lidar_sensor_readings = lidar_sensor_readings[83:len(lidar_sensor_readings)-83]
 
    for i, rho in enumerate(lidar_sensor_readings):
        alpha = lidar_offsets[i]
 
        if rho > LIDAR_SENSOR_MAX_RANGE:
            continue
 
        # The Webots coordinate system doesn't match the robot-centric axes we're used to
        rx = math.cos(alpha)*rho
        ry = -math.sin(alpha)*rho
 
        t = pose_theta + np.pi/2.
        # Convert detection from robot coordinates into world coordinates
        wx =  math.cos(t)*rx - math.sin(t)*ry + pose_x
        wy =  math.sin(t)*rx + math.cos(t)*ry + pose_y
 
        ################ ^ [End] Do not modify ^ ##################
 
        #print("Rho: %f Alpha: %f rx: %f ry: %f wx: %f wy: %f" % (rho,alpha,rx,ry,wx,wy))
        if wx >= 12:
            wx = 11.999
        if wy >= 12:
            wy = 11.999
        if rho < LIDAR_SENSOR_MAX_RANGE:
            # Part 1.3: visualize map gray values.
 
            # You will eventually REPLACE the following lines with a more robust version of the map
            # with a grayscale drawing containing more levels than just 0 and 1.
            # Convert world coords (meters) to map/display cells (30 px per meter)
            mx = min(max(360 - abs(int(wx*30)), 0), 359)
            my = min(max(abs(int(wy*30)), 0), 359)
 
            # Part 1.2: each hit is *evidence*, not proof. Add a little, cap at 1.
            map[mx, my] = min(map[mx, my] + MAP_INCREMENT, 1.0)
 
            # Part 1.3: draw the cell as a gray level (0 = black, 1 = white)
            g = map[mx, my]
            c = int(g * 255)
            display.setColor(c*256**2 + c*256 + c)
            display.drawPixel(mx, my)
 
    # Draw the robot's current pose on the 360x360 display
    display.setColor(int(0xFF0000))
    display.drawPixel(360-abs(int(pose_x*30)), abs(int(pose_y*30)))
 
    ###################
    #
    # Controller
    #
    ###################
    if mode == 'manual':
        # Part 1.1: teleoperation with the arrow keys
        key = keyboard.getKey()
        while keyboard.getKey() != -1: pass   # drain extra queued keys
        if key == keyboard.LEFT:
            vL = -MAX_SPEED / 2
            vR =  MAX_SPEED / 2
        elif key == keyboard.RIGHT:
            vL =  MAX_SPEED / 2
            vR = -MAX_SPEED / 2
        elif key == keyboard.UP:
            vL = MAX_SPEED
            vR = MAX_SPEED
        elif key == keyboard.DOWN:
            vL = -MAX_SPEED
            vR = -MAX_SPEED
        elif key == ord(' '):
            vL = 0
            vR = 0
        elif key == ord('S'):
            # Part 1.4: threshold the evidence map and save it
            map_thresh = (map > MAP_THRESHOLD).astype(np.float32)
            np.save("map.npy", map_thresh)
            print("Map saved to map.npy (%d occupied cells)" % int(map_thresh.sum()))
        elif key == ord('L'):
            # Reload a saved map and redraw it so you can check what was stored
            map = np.load("map.npy").astype(float)
            display.setColor(0x000000)
            display.fillRectangle(0, 0, 360, 360)
            display.setColor(0xFFFFFF)
            for (lx, ly) in np.argwhere(map > 0):
                display.drawPixel(int(lx), int(ly))
            print("Map loaded from map.npy")
        else:
            # No key held: coast to a stop
            vL *= 0.75
            vR *= 0.75
    elif mode == 'autonomous':
        # Part 2: autonomous wall following (not implemented yet)
        pass
    # Odometry code. Don't change vL or vR speeds after this line.
    # We are using GPS and compass for this lab to get a better pose but this is how you'll do the odometry
    pose_x += (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0*math.cos(pose_theta)
    pose_y -= (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0*math.sin(pose_theta)
    pose_theta += (vR-vL)/AXLE_LENGTH/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0
 
    # print("X: %f Z: %f Theta: %f" % (pose_x, pose_y, pose_theta))
 
    # Actuator commands
    robot_parts[MOTOR_LEFT].setVelocity(vL)
    robot_parts[MOTOR_RIGHT].setVelocity(vR)
    
while robot.step(timestep) != -1:
    # there is a bug where webots have to be restarted if the controller exits on Windows
    # this is to keep the controller running
    pass