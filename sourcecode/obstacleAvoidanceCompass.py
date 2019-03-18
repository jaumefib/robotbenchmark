"""Braitenberg-based obstacle-avoiding robot controller."""

from controller import Robot
# import Compass module
from controller import Compass


# Get reference to the robot.
robot = Robot()

# Get simulation step length.
timeStep = int(robot.getBasicTimeStep())

# get robot's Compass device
compass = robot.getCompass("compass")
# enable the Compass
compass.enable(timeStep)

# Constants of the Thymio II motors and distance sensors.
maxMotorVelocity = 9.53  # 24
distanceSensorCalibrationConstant = 360 #360

# Get left and right wheel motors.
leftMotor = robot.getMotor("motor.left")
rightMotor = robot.getMotor("motor.right")

# Get frontal distance sensors.
outerLeftSensor = robot.getDistanceSensor("prox.horizontal.0")
centralLeftSensor = robot.getDistanceSensor("prox.horizontal.1")
centralSensor = robot.getDistanceSensor("prox.horizontal.2")
centralRightSensor = robot.getDistanceSensor("prox.horizontal.3")
outerRightSensor = robot.getDistanceSensor("prox.horizontal.4")

# Enable distance sensors.
outerLeftSensor.enable(timeStep)
centralLeftSensor.enable(timeStep)
centralSensor.enable(timeStep)
centralRightSensor.enable(timeStep)
outerRightSensor.enable(timeStep)

# Disable motor PID control mode.
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

# Set ideal motor velocity.
initialVelocity = 0.95 * maxMotorVelocity # 0.7 # 90%

# Set the initial velocity of the left and right wheel motors.
leftMotor.setVelocity(initialVelocity)
rightMotor.setVelocity(initialVelocity)


while robot.step(timeStep) != -1:
    # Read values from four distance sensors and calibrate.
    outerLeftSensorValue = outerLeftSensor.getValue() / distanceSensorCalibrationConstant
    centralLeftSensorValue = centralLeftSensor.getValue() / distanceSensorCalibrationConstant
    centralSensorValue = centralSensor.getValue() / distanceSensorCalibrationConstant
    centralRightSensorValue = centralRightSensor.getValue() / distanceSensorCalibrationConstant
    outerRightSensorValue = outerRightSensor.getValue() / distanceSensorCalibrationConstant
    
    # to read values
    values = compass.getValues()
    #print(values)
    
    # 2 laser zones
    interior = 1  # good 2, nearly good 1
    exterior = 1  # good 2, nearly good 1
    LeftSensor  = (outerLeftSensorValue/exterior + centralLeftSensorValue*interior)
    RightSensor = (outerRightSensorValue/exterior + centralRightSensorValue*interior) - centralSensorValue*interior
    #print("rs: " + str(RightSensor) + " ls: " + str(LeftSensor))
    
    if LeftSensor == 0 and RightSensor == 0:
        m = 2 # multiplicat 1
        LeftSensor = LeftSensor + (values[0]**m)/values[0]
    
    # Set wheel velocities based on sensor values, prefer right turns if the central sensor is triggered.
    #leftMotor.setVelocity(initialVelocity - (centralRightSensorValue + outerRightSensorValue) / 2)
    #rightMotor.setVelocity(initialVelocity - (centralLeftSensorValue + outerLeftSensorValue) / 2 - centralSensorValue)
    
    # velocity of the left and right motor
    Lvel = max(-9.53, min(initialVelocity - RightSensor , maxMotorVelocity))
    Rvel = max(-9.53, min(initialVelocity - LeftSensor  , maxMotorVelocity))
    #print("lv: " + str(Lvel) + " rv: " + str(Rvel))
    leftMotor.setVelocity(Lvel)
    rightMotor.setVelocity(Rvel)
    
    
    
