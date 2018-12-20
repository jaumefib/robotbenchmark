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

maxSpeed = 80
driver = Driver()
driver.setSteeringAngle(0.0)  # go straight

# get and enable the distance sensors
for name in sensorsNames:
    sensors[name] = driver.getDistanceSensor('distance sensor ' + name)
    sensors[name].enable(10)

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

# Emmagatzema la distancia que ens hem apartat cap a l'esquerra
# El valor de steering cap a l'esquerra es negatiu i cap a la dreta positiu
dLeft = 0

while driver.step() != -1:
    
    # Distance value of sensors
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
    
    speed = maxSpeed * fD / fR
    driver.setCruisingSpeed(speed)
    # brake if we need to reduce the speed
    speedDiff = driver.getCurrentSpeed() - speed
    driver.setBrakeIntensity(max(min(speedDiff / speed, 1), 0))
    
    steer =  0
    steer = min(max(steer, -1), 1)
    dLeft = dLeft + steer
    driver.setSteeringAngle(steer)
    
    #print(lD/lR, fLeft, fD/fR, fRight, rD/rR)
    print(lD/lR)
    print(steer)
    #print(speed)
    
