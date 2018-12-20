"""Sample Webots controller for highway driving benchmark."""

from vehicle import Driver

# name of the available distance sensors
sensorsNames = [
    'front',
    'front right 0',
    'front right 1',
    'front right 2',
    'front left 0',
    'front left 1',
    'front left 2',
    'rear',
    'rear left',
    'rear right',
    'right',
    'left']
sensors = {}

maxSpeed = 80 #80
driver = Driver()
driver.setSteeringAngle(0.0)  # go straight

# get and enable the distance sensors
for name in sensorsNames:
    sensors[name] = driver.getDistanceSensor('distance sensor ' + name)
    sensors[name].enable(10)

# get and enable the GPS
gps = driver.getGPS('gps')
gps.enable(10)

# get the camera
camera = driver.getCamera('camera')
# uncomment those lines to enable the camera
camera.enable(50)
camera.recognitionEnable(50)

# Range of sensors
fR = sensors['front'].getMaxValue()
lR = sensors['left'].getMaxValue()
rR = sensors['right'].getMaxValue()
fR0R = sensors['front right 0'].getMaxValue()
fR1R = sensors['front right 1'].getMaxValue()
fR2R = sensors['front right 2'].getMaxValue()
fL0R = sensors['front left 0'].getMaxValue()
fL1R = sensors['front left 1'].getMaxValue()
fL2R = sensors['front left 2'].getMaxValue()

# Param de optimitzacio
gamma1 = 0.2
gamma2 = 0.1
alpha1 = 0.2
alpha2 = 0.2

while driver.step() != -1:
    # adjust the speed according to the value returned by the front distance sensor
    fD = sensors['front'].getValue()
    lD = sensors['left'].getValue()
    rD = sensors['right'].getValue()
    fR0D = sensors['front right 0'].getValue()
    fR1D = sensors['front right 1'].getValue()
    fR2D = sensors['front right 2'].getValue()
    fL0D = sensors['front left 0'].getValue()
    fL1D = sensors['front left 1'].getValue()
    fL2D = sensors['front left 2'].getValue()
    
    fLeft = fL0D/fL0R + fL1D/fL1R + fL2D/fL2R
    fRight = fR0D/fR0R + fR1D/fR1R + fR2D/fR2R 
    
    speed = min(maxSpeed * fD / fR, maxSpeed * (fLeft+fRight))
    driver.setCruisingSpeed(min(speed, 200))
    
    # brake if we need to reduce the speed because there is a car in front
    speedDiff = driver.getCurrentSpeed() - speed
    # steer left if there is no car in the left
    steer = (fD/fR - lD/lR - fLeft*alpha1)*gamma1 + (-fD/fR + rD/rR + fRight*alpha2)*gamma2
    print(lD/fR, fLeft, fD/lR, fRight, rD/rR)
    print(steer)
    print(speed)
    
    speed = max(min(speedDiff/speed,1), 0)
    
    driver.setBrakeIntensity(speed)
    driver.setSteeringAngle(steer) # si steer positiu gir a ma dreta, si negatiu gir a ma esquerra
