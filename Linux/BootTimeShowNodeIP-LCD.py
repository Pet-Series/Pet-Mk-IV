#!/usr/bin/env python
# Github @SeniorKullken 2020-01-04
#
# Show Linux/Raspian status/information on LCD-Display (SSD1306)
# Row1: Show local HostName = HostName 
# Row2: Show local IP-address = IP
#
# -----Prerequisite------------------------------
#  Script is supposed to run once, like via /etc/rc.local
# -----Prerequisite------------------------------
#  Need to install the following Python lib.
#  $ sudo pip3 install Adafruit-SSD1306
#  $ sudo pip install Adafruit-SSD1306
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

# View Nodename on LCD row #1
# Get local Node/HostName and IP-adress (behind a NAT). 
hostName=socket.gethostname()
lcd.text(hostName, 1, 'center')

# View IP-address on LCD row #2
# A better algorithm to retrieve IP-address due to complexity with multiple IP and subnet behind NAT.
# IP = (([ip for ip in socket.gethostbyname_ex(socket.gethostname())[2] if not ip.startswith("127.")] or [[(s.connect(("8.8.8.8", 53)), s.getsockname()[0], s.close()) for s in [socket.socket(socket.AF_INET, socket.SOCK_DGRAM)]][0][1]]) + ["no IP found"])[0]

count = 0
max_count = 20
IP = ""

while True:
    count+=1
    IP = subprocess.check_output(["hostname", "-I"]).decode('utf-8').strip()

    #print ("Count=" + str(count) + " IP=<" + IP + ">") # Logg text on console
    
    if IP:
        # IP detected. THen show IP address on display and break the loop.
        lcd.text(IP, 2, 'center')             # Logg text on display
        print ("IP=<" + IP + "> <-IP found") # Logg text on console
        break
    else:
        # No IP detected. Then show intermediate message and try again.
        lcd.text("No IP. Count=" + str(count), 2, 'center')        # Logg text on display
        print   ("No IP. Count=" + str(count) + " <-NO IP FOUND!") # Logg text on console
        time.sleep(2)
 
    if count >= max_count:
        # No IP detected whin max_cont. Then show "Warning" message on display and break the loop.        
        lcd.text("No IP. Timeout..", 2, 'center')                            # Logg text on display
        print   ("No IP. Timeout.. Count=" + str(count) + " <-NO IP FOUND!") # Logg text on console
        break

# debug feature for interactive test of the script.
# If parameter == '-clear' then overwrite existing text on LCD-display
if len(sys.argv) > 1 and (sys.argv[1] == '--clear' or sys.argv[1] == '-c'):
    lcd.text(" -- ", 1, 'center')
    lcd.text(" :-)", 2, 'center')

# Leave a logg-text on the console/terminal
# "2019-10-19 Clock=19:59:38.812054 [LinuxStatusOnLCD-display.py] Done"
#print (str(datetime.datetime.now().date()) + " Clock=" + str(datetime.datetime.now().time()) + " [" + sys.argv[0] + "] Done")