import threading as thr
import RPi.GPIO as GPIO
import time

class DistanceThread (thr.Thread):
    def __init__ (self, size=20):
        thr.Thread.__init__(self, daemon=True)
        self.size = size
        self.buffer = [-1] * size
        self.writeIndex = 0
        self.lock = thr.Lock()
        self.notDone = True

        #
        # +-------+--------------+------+
        # |Module | Desc GPIO    |      |
        # |PCB    | Header       |Pins  |
        # +-------+--------------+------+
        # |GROUND | Ground	      |P1-06 |
        # |TRIGGER| GPIO23       |P1-16 |
        # |ECHO   | GPIO24       |P1-18 |
        # |       | Conv. 5V->3V |      |
        # |VCC	   | 5V	          |P1-02 |
        # +-------+--------------+------+
        #
        # Use BCM GPIO references
        # instead of physical pin numbers
        GPIO.setmode(GPIO.BCM)
        # To Avoid - "RuntimeWarning: This channel is already in use, continuing anyway."
        GPIO.setwarnings(False)

        # Define GPIO to use on Pi
        self.GPIO_TRIGGER = 7  #Dist Sensor 1
        self.GPIO_ECHO    = 5  #Dist Sensor 1
##        self.GPIO_TRIGGER = 12  # DistSensor 2
##        self.GPIO_ECHO    = 16  # Distsensor 2
##        self.GPIO_TRIGGER = 20  # DistSensor 3
##        self.GPIO_ECHO    = 21  # Distsensor 3

        # Set pins as output and input
        GPIO.setup(self.GPIO_TRIGGER,GPIO.OUT)  # Trigger
        GPIO.setup(self.GPIO_ECHO,GPIO.IN)      # Echo

        # Set trigger to False (Low)
        GPIO.output(self.GPIO_TRIGGER, False)

    def run(self):
       while self.notDone:
           measurement = self.measure()
           self.write(measurement)

    def write(self, value):
        self.lock.acquire()
        self.buffer[self.writeIndex] = value
        self.writeIndex += 1
        if self.writeIndex >= self.size:
            self.writeIndex = 0
        self.lock.release()

    def read(self):
        retList = []
        self.lock.acquire()
        for i in range(self.writeIndex-1, -1, -1):
            retList.append(self.buffer[i])
        for i in range(self.size-1, self.writeIndex-1, -1):
            retList.append(self.buffer[i])
        self.lock.release()
        return retList

    def measure(self):
        # This function measures a distance
        GPIO.output(self.GPIO_TRIGGER, True)
        time.sleep(0.00001)
        GPIO.output(self.GPIO_TRIGGER, False)
        funcStart = time.time()
        pulsStart = funcStart

        while GPIO.input(self.GPIO_ECHO)==0:
            pulsStart = time.time()
            if pulsStart-funcStart > 0.2:
                return -2
            time.sleep(0.0001)

        pulsStop = pulsStart
        while GPIO.input(self.GPIO_ECHO)==1:
            pulsStop = time.time()
            time.sleep(0.0001)
        
        pulsDuration = pulsStop - pulsStart # Calculate pulse duration
        distance = (pulsDuration * 17150) # 34300/2

        return distance

    def exit(self):
        self.notDone = False















        
        
        
