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

# Variable for not using compass
drift = 0

while robot.step(timeStep) != -1:
    # Read values from four distance sensors and calibrate.
    outerLeftSensorValue = outerLeftSensor.getValue() / distanceSensorCalibrationConstant
    centralLeftSensorValue = centralLeftSensor.getValue() / distanceSensorCalibrationConstant
    centralSensorValue = centralSensor.getValue() / distanceSensorCalibrationConstant
    centralRightSensorValue = centralRightSensor.getValue() / distanceSensorCalibrationConstant
    outerRightSensorValue = outerRightSensor.getValue() / distanceSensorCalibrationConstant
    
    interior = 3
    exterior = 1
    LeftSensor = (centralLeftSensorValue*interior + outerLeftSensorValue*exterior) + centralSensorValue*interior
    RightSensor = (centralRightSensorValue*interior + outerRightSensorValue*exterior)
        
    alpha1 = 0.05
    alpha2 = 0.9
    alpha3 = 0.01
    beta1 = 0.9
    beta2 = 0.02
    beta3 = 0.01
    Lvel = alpha1*LeftSensor + alpha2*RightSensor - alpha3*drift
    Rvel = beta1*LeftSensor + beta2*RightSensor + beta3*drift
    
    print("L: "+ str(Lvel) + " R: " + str(Rvel))
    
    gamma = 0.2
    drift = drift + gamma*(Lvel - Rvel)
    
    # Set wheel velocities based on sensor values, prefer right turns if the central sensor is triggered.
    leftMotor.setVelocity(max(-9.53, min(initialVelocity - Lvel,maxMotorVelocity)))
    rightMotor.setVelocity(max(-9.53, min(initialVelocity - Rvel,maxMotorVelocity)))
    
    
