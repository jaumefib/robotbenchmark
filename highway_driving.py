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

maxSpeed = 80 # 80
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
rLR = sensors['rear left'].getMaxValue()
rRR = sensors['rear right'].getMaxValue()


# Emmagatzema la distancia que ens hem apartat cap a l'esquerra
# El valor de steering cap a l'esquerra es negatiu i cap a la dreta positiu
dLeft = 0
dRight = 0.576310396194

# retornar a la posicio inicial
gamma = 0.2

# max steer
maxSteer = 0.1
# Evitar canvis bruscos
lastSteer = 0

# Configuracions
alpha1 = 0.8
alpha2 = 0.5
alpha3 = 0.5
alpha4 = 0.1
alpha5 = 0.1
beta = 1
phi = 0.2

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
    rLD = sensors['rear left'].getValue()
    rRD = sensors['rear right'].getValue()
    fLeft = fL0D/fL0R + fL1D/fL1R + fL2D/fL2R
    fRight = fR0D/fR0R + fR1D/fR1R + fR2D/fR2R 
    
    speed = maxSpeed * (fD/fR) - fLeft - fRight - (lD/lR) - (rD/rR)
    driver.setCruisingSpeed(speed)
    # brake if we need to reduce the speed
    speedDiff = driver.getCurrentSpeed() - speed
    driver.setBrakeIntensity(max(min(speedDiff / speed, 1), 0))
    
    steer = (-1+fD/fR-phi)*((dLeft*gamma+(lD/lR)+fLeft)*alpha2 + (dRight-(rD/rR)-fRight)*alpha3 + (rLD/rLR)*alpha4 - (rRD/rRR)*alpha5) 
    steer = min(max(steer, -maxSteer), maxSteer)
    steer = (steer-lastSteer)/2
    lastSteer = steer
    dLeft = 1 - (lD/lR)*beta
    driver.setSteeringAngle(steer)
    
    '''
    print 'Left: %(l).2f  FrontLeft: %(fl).2f Front = %(f).2f FrontRight = %(fr).2f Right: %(r).2f' % {
    'l': lD/lR,
    'fl': fLeft,
    'f': fD/fR,
    'fr': fRight,
    'r': rD/rR,
    }
    '''
    #print("Steer: %.2f" % (steer))
    #print("dLeft: %.2f" % (dLeft))
    #print("Speed: %.2f" % (speed))
    
    
