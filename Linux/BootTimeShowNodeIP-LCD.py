#!/usr/bin/env python
# Show Linux/Raspian status/information on OLED-Display (SSD1306)
# Script run once... Supposted to be launched via crontab.
# $ sudo pip3 install Adafruit-SSD1306
# $ sudo pip install Adafruit-SSD1306
#
# Get local HostName = HostName 
# Get locval IP-adfress = IP
# Get CPU-load = CPU
# Get TotaltMemry/UsedMemory info. = MemUsage
# Get TotalStorage/UsedStorage info. = Disk
# 
from rpi_lcd import LCD
import socket
import datetime
import sys
import subprocess
import time

#Initiate LCD-display
lcd = LCD(address=0x3f, width=16, rows=2)

# View LCD row #1
# Get local Node/HostName and IP-adress (behind a NAT). 
hostName=socket.gethostname()
lcd.text(hostName, 1, 'center')

# View LCD row #2
# A better algorithm to retrieve IP-address due to complexity with multiple IP and subnet behind NAT.
# IP = (([ip for ip in socket.gethostbyname_ex(socket.gethostname())[2] if not ip.startswith("127.")] or [[(s.connect(("8.8.8.8", 53)), s.getsockname()[0], s.close()) for s in [socket.socket(socket.AF_INET, socket.SOCK_DGRAM)]][0][1]]) + ["no IP found"])[0]

count = 0
IP = ""

while not IP:
    count+=1
    IP = subprocess.check_output(["hostname", "-I"]).decode('utf-8').strip()
    print ("Count=" + str(count) + " IP=<" + IP + ">")
    if not IP:
        lcd.text("-No IP. Count=" + str(count) + " -", 2, 'center')
        time.sleep(2)
    lcd.text(IP, 2, 'center')

# If parameter == '-clear' then overwrite existing text on LCD-display
if len(sys.argv) > 1 and sys.argv[1] == '-clear':
    lcd.text(" -- ", 1, 'center')
    lcd.text(" :-)", 2, 'center')

# Leave a logg-text on the console/terminal
# "2019-10-19 Clock=19:59:38.812054 [LinuxStatusOnLCD-display.py] Done"
print (str(datetime.datetime.now().date()) + " Clock=" + str(datetime.datetime.now().time()) + " [" + sys.argv[0] + "] Done")