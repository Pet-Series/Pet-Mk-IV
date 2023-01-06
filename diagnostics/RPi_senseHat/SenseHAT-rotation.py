import time

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

sense = SenseHat()
sense.clear()
compass_enabled = False #	Boolean 	True False 	Whether or not the compass should be enabled.
gyro_enabled = True #	Boolean 	True False 	Whether or not the gyroscope should be enabled.
accel_enabled = False #	Boolean 	True False 	Whether or not the accelerometer should be enabled.
sense.set_imu_config(compass_enabled, gyro_enabled, accel_enabled)

# Visa bokstav("pil") på 8x8 Display/Matrix
sense.show_letter("V")

try:
    print ("Try...")
    while True:

        o = sense.get_orientation()
        pitchRaw = o["pitch"]
        rollRaw = o["roll"]
        yawRaw = o["yaw"]
    
        pitch = round(pitchRaw,0)
        roll  = round(rollRaw ,0)
        yaw   = round(yawRaw  ,0)
        print("pitch {0} roll {1} yaw {2}".format(pitch, roll, yaw))

        if (yaw >= 315.0 or yaw < 45.0):
            sense.show_letter("V")
            sense.set_rotation(90)
        elif (yaw >= 45.0 and yaw < 135.0 ):
            sense.show_letter("V")
            sense.set_rotation(0)
        elif (yaw >= 135.0 and yaw < 225.0 ):
            sense.show_letter("V")
            sense.set_rotation(270)
        elif (yaw >= 225.0 and yaw <= 315.0 ):
            sense.show_letter("V")
            sense.set_rotation(180)
        else:
            sense.show_letter("?")
            
        time.sleep(0.1)

except KeyboardInterrupt:
    # User pressed CTRL-C
    print ("Cancel! Ctrl-C pressed...")
    sense.clear()
