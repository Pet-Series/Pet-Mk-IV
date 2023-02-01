from sense_hat import SenseHat
from time import sleep
# Snurra på golvet/bordet och läs rotation runt yaw("gira")axeln.
# + kombinera det med att läsa av fröjdepinnen("joystick").
# ...men här håller olika delar på och skriver sönder/över varandra på 8x8 displayen.
# ...knapparna är tilldelade lite hit och dit ;-(

## https://pythonhosted.org/sense-hat/api/#imu-sensor
##Kolla vilka i2c enheter som är inkopplade just nu 
##$ sudo i2cdetect -y 1
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

# Define the functions
def red():
  sense.clear(255, 0, 0)

def blue():
  sense.clear(0, 0, 255)

def green():
  sense.clear(0, 255, 0)
  
def yellow():
  sense.clear(255, 255, 0)

# Tell the program which function to associate with which direction
sense.stick.direction_up = red
sense.stick.direction_down = blue
sense.stick.direction_left = green
sense.stick.direction_right = yellow
sense.stick.direction_middle = sense.clear   # Press the enter key

try:
    # Denna kod kör bara en gång!
    # Visa tecken på 8x8 Display/Matrix
    sense.show_letter("1") 
    event = sense.stick.wait_for_event()
    sleep(0.1)
    event = sense.stick.wait_for_event(emptybuffer=True)
    sense.clear()
    
    sense.show_letter("2") 
    pressure = round (sense.get_pressure(),1)
    print("Pressure: %s Millibars" % pressure)
    temp = round (sense.get_temperature(),1)
    print("Temperature: %s C" % temp)
 
    sense.show_letter("3") 
    event = sense.stick.wait_for_event()
    sleep(0.1)
    event = sense.stick.wait_for_event(emptybuffer=True)
    sense.clear()
    
    # Visa bokstav("pil") på 8x8 Display/Matrix
    sense.show_letter("V")
    event = sense.stick.wait_for_event()
    print ("Try...")
    while True:
        #o = sense.get_orientation_radians() 
        o = sense.get_orientation_degrees()
        pitchRaw = o["pitch"]
        rollRaw = o["roll"]
        yawRaw = o["yaw"]
    
        pitch = round(pitchRaw,0)
        roll  = round(rollRaw ,0)
        yaw   = round(yawRaw  ,0)
        print("pitch {0} roll {1} yaw {2}".format(pitch, roll, yaw))

        if (yaw < 90.0):
            sense.set_rotation(0)
        elif (yaw >= 90.0 and yaw < 180.0 ):
            sense.set_rotation(90)
        elif (yaw >= 180.0 and yaw < 270.0 ):
            sense.set_rotation(180)
        elif (yaw >= 270.0 and yaw < 360.0 ):
            sense.set_rotation(270)
        else:
            sense.show_letter("?") # Detta kan aldrig hända!!!

except KeyboardInterrupt:
    # User pressed CTRL-C
    # Reset GPIO settings
    print ("Cancel! Ctrl-C pressed...")
    sense.clear()
