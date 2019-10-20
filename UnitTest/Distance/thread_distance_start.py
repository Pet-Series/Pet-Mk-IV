import time
import RPi.GPIO as GPIO

from distance_thread import DistanceThread
import movement as mv

##pi@raspiKull3:~ $ gpio readall
## +-----+-----+---------+------+---+---Pi 3---+---+------+---------+-----+-----+
## | BCM | wPi |   Name  | Mode | V | Physical | V | Mode | Name    | wPi | BCM |
## +-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+
## |     |     |    3.3v |      |   |  1 || 2  |   |      | 5v      |     |     |
## |   2 |   8 |   SDA.1 | ALT0 | 1 |  3 || 4  |   |      | 5v      |     |     |
## |   3 |   9 |   SCL.1 | ALT0 | 1 |  5 || 6  |   |      | 0v      |     |     |
## |   4 |   7 | GPIO. 7 |   IN | 0 |  7 || 8  | 0 | IN   | TxD     | 15  | 14  |
## |     |     |      0v |      |   |  9 || 10 | 1 | IN   | RxD     | 16  | 15  |
## |  17 |   0 | GPIO. 0 |   IN | 0 | 11 || 12 | 0 | IN   | GPIO. 1 | 1   | 18  |
## |  27 |   2 | GPIO. 2 |   IN | 0 | 13 || 14 |   |      | 0v      |     |     |
## |  22 |   3 | GPIO. 3 |   IN | 0 | 15 || 16 | 0 | OUT  | GPIO. 4 | 4   | 23  |
## |     |     |    3.3v |      |   | 17 || 18 | 0 | IN   | GPIO. 5 | 5   | 24  |
## |  10 |  12 |    MOSI | ALT0 | 0 | 19 || 20 |   |      | 0v      |     |     |
## |   9 |  13 |    MISO | ALT0 | 0 | 21 || 22 | 0 | IN   | GPIO. 6 | 6   | 25  |
## |  11 |  14 |    SCLK | ALT0 | 0 | 23 || 24 | 1 | OUT  | CE0     | 10  | 8   |
## |     |     |      0v |      |   | 25 || 26 | 1 | OUT  | CE1     | 11  | 7   |
## |   0 |  30 |   SDA.0 |   IN | 1 | 27 || 28 | 1 | IN   | SCL.0   | 31  | 1   |
## |   5 |  21 | GPIO.21 |  OUT | 0 | 29 || 30 |   |      | 0v      |     |     |
## |   6 |  22 | GPIO.22 |  OUT | 1 | 31 || 32 | 0 | IN   | GPIO.26 | 26  | 12  |
## |  13 |  23 | GPIO.23 |  OUT | 0 | 33 || 34 |   |      | 0v      |     |     |
## |  19 |  24 | GPIO.24 |   IN | 0 | 35 || 36 | 0 | IN   | GPIO.27 | 27  | 16  |
## |  26 |  25 | GPIO.25 |  OUT | 1 | 37 || 38 | 0 | IN   | GPIO.28 | 28  | 20  |
## |     |     |      0v |      |   | 39 || 40 | 0 | IN   | GPIO.29 | 29  | 21  |
## +-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+
## | BCM | wPi |   Name  | Mode | V | Physical | V | Mode | Name    | wPi | BCM |
## +-----+-----+---------+------+---+---Pi 3---+---+------+---------+-----+-----+

##GPIO.cleanup()


thread1 = DistanceThread()
thread1.start()
# Allow time for sensors to obtain initial data.
time.sleep(1)

try:
    print ("Try...")
    while True:
        distance = thread1.read()[0]
        print ('Distance : %.1f' % distance )
##        print(distance)
        if distance > 25:
            mv.Forward(True)
        else:
            mv.Forward(False)
        time.sleep(0.1)
        if distance<0:
            print ("Ett Fel")

except KeyboardInterrupt:
    # User pressed CTRL-C
    # Reset GPIO settings
    print ("Cancel! Ctrl-C pressed...")
    thread1.exit()
    time.sleep(0.3)
    GPIO.cleanup()
