from controller import Robot, DistanceSensor, Motor
from threading import Timer
import time

# time in [ms] of a simulation step
TIME_STEP = 64

MAX_SPEED = 6.28

# create the Robot instance.
robot = Robot()

# initialize devices
ps = []
psNames = [
    'ps0', 'ps1', 'ps2', 'ps3',
    'ps4', 'ps5', 'ps6', 'ps7'
]

for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(TIME_STEP)

ls = []
lsNames = ['ls0', 'ls1', 'ls2', 'ls3', 'ls4', 'ls5', 'ls6', 'ls7']

for i in range(len(lsNames)):
    ls.append(robot.getDevice(lsNames[i]))
    ls[i].enable(TIME_STEP)

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

vL = 0
vR = 1
side = 0
rotateonce = 0

def rotate180():
    print("I'm here")
    leftMotor.setVelocity(-3)
    rightMotor.setVelocity(3)
    state = 0
    while state <= 1500:
        if robot.step(TIME_STEP) == -1:
            return
        state += TIME_STEP
    leftMotor.setVelocity(0.0)
    rightMotor.setVelocity(0.0)
    return
    
def swaplight():
    lightstuff=0
    return

# feedback loop: step simulation until receiving an exit event
while robot.step(TIME_STEP) != -1:
    # read sensors outputs
    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())

    lsValues = []
    for i in range(8):
        lsValues.append(ls[i].getValue())
        
       #0 1 2 5 6 7

    #if left sensor gets low, then have the right actuator speed up or lower in speed to position match
    lightstuff = lsValues[0] < 80.0 and lsValues[1] < 80.0 and lsValues[2] < 80.0 and lsValues[5] < 80.0 and lsValues[6] < 80.0 and lsValues[7] < 80.0
    if side == 0:
        if rotateonce == 0 and lightstuff == 1:
            rotate180()
            side = 1
            rotateonce = 1
        if psValues[7] >= 80 and psValues[0] >= 80:
            vR = 0
            vL = 4
        elif psValues[7] > 80 and psValues[4] < 80:
            vR = 2
            vL = 4
        elif psValues[4] > 80 and psValues[7] < 80:
            vR = 6
            vL = 4
    elif side == 1:
        if rotateonce == 1 and lightstuff == 1:
            rotate180()
            side = 0
            rotateonce = 0
        if psValues[0] >= 80 and psValues[7] >= 80:
            vR = 4
            vL = 0
        elif psValues[0] > 80 and psValues[3] < 80:
            vR = 4
            vL = 2
        elif psValues[3] > 80 and psValues[0] < 80:
            vR = 4
            vL = 6

    # write actuators inputs
    leftMotor.setVelocity(vL)
    rightMotor.setVelocity(vR)