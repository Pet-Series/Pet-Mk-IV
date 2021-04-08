import time

import numpy as np
from pdb import set_trace as bp

from sense_hat import SenseHat
# Snurra på golvet/bordet och läs rotation runt yaw("gira")axeln.
#
# IMU = Inertial Measurement Unit)
#  1. gyroscope
#  2. Accelerometer
#  3. Magnetometer (compass)
#
# Bra o ha länk: https://pythonhosted.org/sense-hat/
# Bra o ha länk: https://pythonhosted.org/sense-hat/api/#imu-sensor
# Bra o ha länk: https://www.raspberrypi.org/documentation/hardware/sense-hat/
#


## https://pythonhosted.org/sense-hat/api/#imu-sensor
## Kolla vilka i2c enheter som är inkopplade just nu 
## $ sudo i2cdetect -y 1
##     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
##00:          -- -- -- -- -- -- -- -- -- -- -- -- -- 
##10: -- -- -- -- -- -- -- -- -- -- -- -- 1c -- -- -- 
##20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
##30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
##40: -- -- -- -- -- -- UU -- -- -- -- -- -- -- -- -- 
##50: -- -- -- -- -- -- -- -- -- -- -- -- 5c -- -- 5f 
##60: -- -- -- -- -- -- -- -- -- -- 6a -- -- -- -- -- 
##70: -- -- -- -- -- -- -- --  


# Setup for IMU-unit.
sense = SenseHat()
sense.clear()
compass_enabled = False #	Boolean: True/False Whether or not the compass should be enabled.
gyro_enabled = True     #	Boolean: True/False Whether or not the gyroscope should be enabled.
accel_enabled = False   #	Boolean: True/False Whether or not the accelerometer should be enabled.
sense.set_imu_config(compass_enabled, gyro_enabled, accel_enabled)

# Visa bokstav("pil") på 8x8 Display/Matrix
sense.show_letter("V")

# Initial position and velocity set to (0,0,0).
pos = np.array((0,0,0))
vel = np.array((0,0,0))

# Timestep
dt = 0.1

def estimateBias(nrSamples=100, dt=dt/10):
    print('Bias measurement in progress. Do not move the unit.')
    samples = np.zeros((nrSamples, 3))
    for i in range(nrSamples):
        accRaw = sense.get_accelerometer_raw() # Seems to measure in 10*m/s^2
        xRaw = accRaw['x']
        yRaw = accRaw['y']
        zRaw = accRaw['z']
        
        acc = np.array((xRaw, yRaw, zRaw))
        acc = acc*10 # Scale to m/s^2 (hopefully...)
        
        samples[i] = acc
        
        time.sleep(dt)
    
    # Compute bias as the mean over all samples.
    bias = samples.mean(axis=0)
    
    print('Bias measurement done.')
    return bias

# Constant bias in accelerometer
bias = estimateBias()

try:
    print ("Try...")
    while True:

        accRaw = sense.get_accelerometer_raw() # Seems to measure in 10*m/s^2
        xRaw = accRaw['x']
        yRaw = accRaw['y']
        zRaw = accRaw['z']
        
        acc = np.array((xRaw, yRaw, zRaw))
        acc = acc*10 # Scale to m/s^2 (hopefully...)
        acc = acc - bias # Constant bias adjustement
        
        velNew = vel + acc*dt
        posNew = pos + vel*dt + (acc*dt**2) / 2
        
        # Print new position
        x = round(posNew[0], 2)
        y = round(posNew[1], 2)
        z = round(posNew[2], 2)
        print(x, y, z)
        
        #x = round(accRaw['x']*10, 1)
        #y = round(accRaw['y']*10, 1)
        #z = round(accRaw['z']*10, 1)
        #print(x, y, z)
        
        vel = velNew
        pos = posNew
            
        time.sleep(dt)

except KeyboardInterrupt:
    # User pressed CTRL-C
    print("Cancel! Ctrl-C pressed...")
    sense.clear()
    
except:
    # Other exception
    print('Error!')
    print('Clearing SenseHat...')
    sense.clear()
    print('...SenseHat cleared.')
    raise