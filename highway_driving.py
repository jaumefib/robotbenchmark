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

maxSpeed = 110 # 80
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

## Variables Lantents
# El valor de steering cap a l'esquerra es negatiu i cap a la dreta positiu
dLeft = 0                # Distancia que ens hem apartat cap a l'esquerra
dRight = 0.58 #0.576310396194  # Distancia original amb la valla de la dreta
maxSteer = 0.4           # Maxim steer que se li permet al cotxe
lastSteer = 0            # Valor de l'ultim steer realitzat pel cotxe

## Configuracions
alpha1 = 0.45  # Importancia de la deteccio esquerra per girar esquerra amb steering
alpha2 = 0.4  # Importancia de la deteccio dreta per girar dreta amb steering
alpha3 = 0.2  # Importancia de la deteccio del darrera esquerra
alpha4 = 0.2  # Importancia de la deteccio del darrera dret 
beta = 1      # importancia de la separacio amb el lateral dret 
phi = 0.1     # si no hi ha ningu al davant poder fer steering
gamma = 0.05   # velocitat de retorn a la posicio inicial
theta = 0.5   # valor per reduir la variacio d'un steering amb el seu anterior, evitar canvis bruscos
# Velocity Hiperparameter
vh1 = 3  # Front Left
vh2 = 3  # Front Right
vh3 = -0.25  # Left
vh4 = 0.25  # Right
vh5 = -0.5  # Rear Left
vh6 = -0.5  # Rear Right

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
    
    speed = pow(maxSpeed,(fD/fR)) - fLeft*vh1 - fRight*vh2 - (lD/lR)*vh3 - (rD/rR)*vh4 - (rLD/rLR)*vh5 - (rRD/rRR)*vh6
    driver.setCruisingSpeed(max(speed,0))
    # brake if we need to reduce the speed
    speedDiff = driver.getCurrentSpeed() - speed
    driver.setBrakeIntensity(max(min(speedDiff / speed, 1), 0))
    
    steer = (-1+fD/fR-phi)*((dLeft*gamma+(lD/lR)+fLeft)*alpha1 + (dRight-(rD/rR)-fRight)*alpha2 + (rLD/rLR)*alpha3 - (rRD/rRR)*alpha4) 
    steer = min(max(steer, -maxSteer), maxSteer)
    steer = (steer-lastSteer)*theta
    lastSteer = steer
    dLeft = 1 - (lD/lR)*beta
    driver.setSteeringAngle(steer)


    if (driver.step()%1000000==0):
        '''
        print 'Left: %(l).2f  FrontLeft: %(fl).2f Front = %(f).2f FrontRight = %(fr).2f Right: %(r).2f' % {
        'l': lD/lR,
        'fl': fLeft,
        'f': fD/fR,
        'fr': fRight,
        'r': rD/rR,
        }
        '''
        print("Steer: %.2f" % (steer))
        #print("dLeft: %.2f" % (dLeft))
        #print("Speed: %.2f" % (speed))
        #print("SpeedDiff: %.2f" % (speedDiff))
