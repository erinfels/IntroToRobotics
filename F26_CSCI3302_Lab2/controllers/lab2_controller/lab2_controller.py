"""csci3302_lab2 controller."""

# You may need to import some classes of the controller module.
import math
import time
from controller import Robot, Motor, DistanceSensor
# import os

# Ground Sensor Measurements under this threshold are black
# measurements above this threshold can be considered white.
# TODO: Set a reasonable threshold that separates "line detected" from "no line detected"
GROUND_SENSOR_THRESHOLD = 500

# These are your pose values that you will update by solving the odometry equations
pose_x = 0
pose_y = 0
pose_theta = 0

# Index into ground_sensors and ground_sensor_readings for each of the 3 onboard sensors.
LEFT_IDX = 0
CENTER_IDX = 1
RIGHT_IDX = 2

# create the Robot instance.
robot = Robot()
state = "speed_measurement"

# ePuck Constants
EPUCK_AXLE_DIAMETER = 0.053  # ePuck's wheels are 53mm apart.
# TODO: set the ePuck wheel speed in m/s after measuring the speed (Part 1)
EPUCK_MAX_WHEEL_SPEED = 0.13643
MAX_SPEED = 6.28
MEASUREMENT_DISTANCE = 0.567

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
ground_sensors = [robot.getDevice('gs0'), robot.getDevice(
    'gs1'), robot.getDevice('gs2')]
for gs in ground_sensors:
    gs.enable(SIM_TIMESTEP)

# Allow sensors to properly initialize
for i in range(10):
    robot.step(SIM_TIMESTEP)

# Initialize variable for left and right speed
# Additional variables for the loop are also forward declared
vL = 0
vR = 0

FORWARD_SPEED = .25 * MAX_SPEED
TURN_SPEED = 0.125 * MAX_SPEED
measurement_start_time = None
start_line_time = None
start_line_confirmed = False
lap_started = False
last_turn = 1

def update_odometry(left_velocity, right_velocity):
    global pose_x, pose_y, pose_theta

    dt = SIM_TIMESTEP / 1000.0

    dL = (left_velocity / MAX_SPEED) * EPUCK_MAX_WHEEL_SPEED * dt
    dR = (right_velocity / MAX_SPEED) * EPUCK_MAX_WHEEL_SPEED * dt

    distance = (dL + dR) / 2.0
    dtheta = (dR - dL) / EPUCK_AXLE_DIAMETER
    theta_mid = pose_theta + dtheta / 2.0

    pose_x += distance * math.cos(theta_mid)
    pose_y += distance * math.sin(theta_mid)
    pose_theta = (pose_theta + dtheta) % (2 * math.pi) 
# Main Control Loop:
while robot.step(SIM_TIMESTEP) != -1:

    # Read ground sensor values
    for i, gs in enumerate(ground_sensors):
        gsr[i] = gs.getValue()

    # TODO: Uncomment to see the ground sensor values!
    # TODO: But when you don't need it, please comment it so you have a clean terminal.
    # print(gsr)

    left_on_line = gsr[LEFT_IDX] < GROUND_SENSOR_THRESHOLD
    center_on_line = gsr[CENTER_IDX] < GROUND_SENSOR_THRESHOLD
    right_on_line = gsr[RIGHT_IDX] < GROUND_SENSOR_THRESHOLD
    all_on_black = left_on_line and center_on_line and right_on_line

    # Integrate the commands used during the step that just finished.
    update_odometry(vL, vR)

    # Part 1
    # TODO: Implement Maximum Speed Measurement under state "speed_measurement"
    # TODO: Save the speed within XZ-plane to EPUCK_MAX_WHEEL_SPEED after measuring it.

    if state == "speed_measurement":
        if measurement_start_time is None:
            measurement_start_time = robot.getTime()

        vL = MAX_SPEED
        vR = MAX_SPEED
        elapsed_time = robot.getTime() - measurement_start_time

        if left_on_line and right_on_line:
            vL = 0.0
            vR = 0.0

            if elapsed_time <= 0.0:
                raise RuntimeError(
                    "Start the robot before the start line to measure speed."
                )

            EPUCK_MAX_WHEEL_SPEED = MEASUREMENT_DISTANCE / elapsed_time

            print(f"Elapsed time: {elapsed_time:.6f} s")
            print(f"Measured speed: {EPUCK_MAX_WHEEL_SPEED:.6f} m/s")

            # Reset the pose at the start line.
            pose_x = 0.0
            pose_y = 0.0
            pose_theta = 0.0

            state = "line_follower"
    # Part 2
    # TODO: Implement Line Following under state "line_follower"
    # TODO: Also implement update_odometry and then call update_odometry here
    # Hints for Line Following:
    #
    # 1) Setting vL=MAX_SPEED and vR=-MAX_SPEED lets the robot turn
    # right on the spot. vL=MAX_SPEED and vR=0.5*MAX_SPEED lets the
    # robot drive a right curve.
    #
    # 2) If your robot "overshoots", turn slower.
    #
    # 3) Only set the wheel speeds once so that you can use the speed
    # that you calculated in your odometry calculation.
    #
    # 4) Disable all console output to simulate the robot superfast
    # and test the robustness of your approach.
    #
    # Hints for update_odometry:
    #
    # 1) Divide vL/vR by MAX_SPEED to normalize, then multiply with
    # the robot's maximum speed in meters per second.
    #
    # 2) SIM_TIMESTEP tells you the elapsed time per step. You need
    # to divide by 1000.0 to convert it to seconds
    #
    # 3) Do simple sanity checks. In the beginning, only one value
    # changes. Once you do a right turn, this value should be constant.
    #
    # 4) Focus on getting things generally right first, then worry
    # about calculating odometry in the world coordinate system of the
    # Webots simulator first (x points down, y points right)
    
    elif state == "line_follower":
        if all_on_black:
            vL = FORWARD_SPEED
            vR = FORWARD_SPEED
        elif left_on_line and not right_on_line:
            last_turn = 1
            if center_on_line:
                vL = 0.5 * FORWARD_SPEED
                vR = FORWARD_SPEED
            else:
                vL = -TURN_SPEED
                vR = TURN_SPEED
        elif right_on_line and not left_on_line:
            last_turn = -1
            if center_on_line:
                vL = FORWARD_SPEED
                vR = 0.5 * FORWARD_SPEED
            else:
                vL = TURN_SPEED
                vR = -TURN_SPEED
        elif center_on_line:
            vL = FORWARD_SPEED
            vR = FORWARD_SPEED
        else:
            vL = -last_turn * TURN_SPEED
            vR = last_turn * TURN_SPEED

    # Part 3
    # TODO: Implement Loop Closure also under state "line_follower" to reset pose when robot passes over the Start Line.
    # Hints:
    #
    # 1) Set a flag whenever you encounter the line
    #
    # 2) Use the pose when you encounter the line last
    # for best results

    if state == "line_follower":
        if all_on_black:
            if start_line_time is None:
                start_line_time = robot.getTime()
            if robot.getTime() - start_line_time > 0.1:
                start_line_confirmed = True
        else:
            # Also count the final interval before leaving the stripe.
            if start_line_time is not None:
                start_line_confirmed = (
                    start_line_confirmed
                    or robot.getTime() - start_line_time > 0.1
                )
            if start_line_confirmed:
                if lap_started:
                    print("Current pose: [%5f, %5f, %5f]" % (pose_x, pose_y, pose_theta))
                   # while (True):
                        #time.sleep(1)

                    print("Lap Started: Pose Reset")

                    pose_x = 0.0
                    pose_y = 0.0
                    pose_theta = 0.0
                    lap_started = True
                else:
                    # The first crossing establishes the reference for one lap.
                    pose_x = 0.0
                    pose_y = 0.0
                    pose_theta = 0.0
                    lap_started = True
            start_line_time = None
            start_line_confirmed = False

    print("Current pose: [%5f, %5f, %5f]" % (pose_x, pose_y, pose_theta))
    #if state == "line_follower":
        #print("Sensors:", gsr, "On line:", (left_on_line, center_on_line, right_on_line), "Motors:", (vL, vR))
    leftMotor.setVelocity(vL)
    rightMotor.setVelocity(vR)