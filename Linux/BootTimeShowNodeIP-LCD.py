#!/usr/bin/env python
# Github @SeniorKullken 2020-01-07
# Github @SeniorKullken 2020-01-04
#
# Show Linux/Raspian status/information on LCD-Display (SSD1306)
# Row1: Show local HostName = HostName 
# Row2: Show local IP-address = IP
#
# -----Prerequisite------------------------------
#  Script is supposed to run once during boot, like via /etc/rc.local
# - Avoid lock the boot process by adding a traling " &"
# /home/pi/ros_ws/src/Pet-Mk-IV/Linux/BootTimeShowNodeIP-LCD.py &
#
#  Script is supposed to run once, like via /etc/rc.local
# -----Prerequisite------------------------------
#  Need to install the following Python lib.
#  $ sudo pip3 install Adafruit-SSD1306
#  $ sudo pip install Adafruit-SSD1306
#
# -----Prerequisite------------------------------
#   $ sudo i2cdetect -y 1
#     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
#  00:          -- -- -- -- -- -- -- -- -- -- -- -- -- 
#  10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
#  20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
#  30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 3f 
#  40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
#  50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
#  60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
#  70: -- -- -- -- -- -- -- --  
#  -----------------------------------
from rpi_lcd import LCD
import socket
import datetime
import sys
import subprocess
import time

#Initiate the SSD1306 LCD-display
lcd = LCD(address=0x3f, width=16, rows=2)

verboseHeader = "[" + sys.argv[0] + "]"

# View Nodename on LCD row #1
# Get local Node/HostName and IP-adress (behind a NAT). 
host_name=socket.gethostname()
lcd.text(host_name, 1, 'center')                       # Logg text on display
print (verboseHeader + "hostname=<" + host_name + ">") # Logg text on console/terminal.

# View IP-address on LCD row #2  <- Must wait for the WiFi to retrive the IP-adress.
#
# An other(better?) algorithm to retrieve IP-address due to complexity with multiple IP and subnet behind NAT.
# View IP-address on LCD row #2
# A better algorithm to retrieve IP-address due to complexity with multiple IP and subnet behind NAT.
# IP = (([ip for ip in socket.gethostbyname_ex(socket.gethostname())[2] if not ip.startswith("127.")] or [[(s.connect(("8.8.8.8", 53)), s.getsockname()[0], s.close()) for s in [socket.socket(socket.AF_INET, socket.SOCK_DGRAM)]][0][1]]) + ["no IP found"])[0]
IP = ""
count = 0
retry_interval = 2 # Wait two seconds before we try to read the IP address again
max_count = 20     # Max retries (in total we wait 40(2sec*20 attenpt) seconds before giving up..


while True:
    count+=1
    IP = subprocess.check_output(["hostname", "-I"]).decode('utf-8').strip()
 #   IP = "" # Debug - Simulate no IP/WiFi connection...
    if IP:
        # IP detected. THen show IP address on display and break the loop.
        lcd.text(IP, 2, 'center')                            # Logg text on display
        print (verboseHeader + "IP=<" + IP + "> <-IP found") # Logg text on console
        break
    else:
        # No IP detected. Then show intermediate message and try again.
        lcd.text("No IP. Count=" + str(count), 2, 'center')                        # Logg text on display
        print   (verboseHeader + "No IP. Count=" + str(count) + " <-NO IP FOUND!") # Logg text on console
        time.sleep(retry_interval)
 
    if count >= max_count:
        # No IP detected whin max_cont. Then show "Warning" message on display and break the loop.        
        print   (verboseHeader + "No IP. Timeout.. After #" + str(count) + " attempts") # Logg text on console
        lcd.text("No IP. Timeout..", 2, 'center')                                       # Logg text on display
        break

# debug feature for interactive test of the script.
# If parameter == '-clear' then overwrite existing text on LCD-display
if len(sys.argv) > 1 and (sys.argv[1] == '--clear' or sys.argv[1] == '-c'):
    lcd.text(" -- ", 1, 'center')
    lcd.text(" :-)", 2, 'center')

# Leave a logg-text on the console/terminal
# "2019-10-19 Clock=19:59:38.812054 [LinuxStatusOnLCD-display.py] Done"
# print (verboseHeader + str(datetime.datetime.now().date()) + " Clock=" + str(datetime.datetime.now().time()) +" <- Done")
