#!/usr/bin/env python
# Github @SeniorKullken 2020-01-04
#
# Show Linux/Raspian status/information on LCD-Display (SSD1306)
# Row1: Show X Y 
# Row2: Show Z Z
#
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
import argparse

parser = argparse.ArgumentParser(description='Show system status on a LCD-display')
parser.add_argument("--row1", "-1", default="host_name", type=str, help="What to show on row #1",
                    choices=["host_name", "CPU_load", "mem_usage", "disk_usage", "IP"])
parser.add_argument("--row2", "-2", default="IP", type=str, help="What to show on row #2",
                    choices=["host_name", "CPU_load", "mem_usage", "disk_usage", "IP"])
parser.add_argument("--clear", "-c", action="store_true", help="Clear row1 & row2")
args = parser.parse_args()



#Initiate the SSD1306 LCD-display
lcd = LCD(address=0x3f, width=16, rows=2)

# Get local Node/HostName and IP-adress (behind a NAT). 
host_name=socket.gethostname()

# Get CPU-load = CPU
cmd = "top -bn1 | grep load | awk '{printf \"%.2f%%\", $(NF-2)}'"
CPU_load = subprocess.check_output(cmd, shell = True ).decode('utf-8')
    
# Get TotaltMemry/UsedMemory information
#cmd = "free -m | awk 'NR==2{printf \"%s/%sMB %.2f%%\", $3,$2,$3*100/$2 }'" #"Total/Used Procent%"
cmd = "free -m | awk 'NR==2{printf \"%s/%sMB %.0f%%\", $3,$2,$3*100/$2 }'" #"Total/Used Procent%"
mem_usage = subprocess.check_output(cmd, shell = True ).decode('utf-8')
    
#Get TotalStorage/UsedStorage information
cmd = "df -h | awk '$NF==\"/\"{printf \"%d/%dGB %s\", $3,$2,$5}'"  #"Total/Used Procent%"
disk_usage = subprocess.check_output(cmd, shell = True ).decode('utf-8')

# Get IP-address on LCD 
# A better algorithm to retrieve IP-address due to complexity with multiple IP and subnet behind NAT.
# IP = (([ip for ip in socket.gethostbyname_ex(socket.gethostname())[2] if not ip.startswith("127.")] or [[(s.connect(("8.8.8.8", 53)), s.getsockname()[0], s.close()) for s in [socket.socket(socket.AF_INET, socket.SOCK_DGRAM)]][0][1]]) + ["no IP found"])[0]
IP = subprocess.check_output(["hostname", "-I"]).decode('utf-8').strip()


# Select what to show on each row
if args.clear:
    print("Clear screen")
    lcd.text(" -- ", 1, 'center')
    lcd.text(" :-)", 2, 'center')
else:
    print("Not clear all")
    
if args.row1 == "host_name":
    print("Row1 = Show 'host_name'")
    lcd.text(host_name, 1, 'center')
elif args.row1 == "CPU_load":
    print("Row1 = Show 'CPU_load'")
    lcd.text(CPU_load, 1, 'right')
elif args.row1 == "mem_usage":
    print("Row1 = Show 'mem_usage'")
    lcd.text(mem_usage, 1, 'right')
elif args.row1 == "disk_usage":
    print("Row1 = Show 'disk_usage'")
    lcd.text(disk_usage, 1, 'right')
elif args.row1 == "IP":
    print("Row1 = Show 'IP'")
    lcd.text(IP, 1, 'center')
else:
    print("Row1 = ...invalid!")

if args.row2 == "host_name":
    print("Row2 = Show 'host_name'")
    lcd.text(host_name, 2, 'center')
elif args.row2 == "CPU_load":
    print("Row2 = Show 'CPU_load'")
    lcd.text(CPU_load, 2, 'right')
elif args.row2 == "mem_usage":
    print("Row2 = Show 'mem_usage'")
    lcd.text(mem_usage, 2, 'right')
elif args.row2 == "disk_usage":
    print("Row2 = Show 'disk_usage'")
    lcd.text(disk_usage, 2, 'right')
elif args.row2 == "IP":
    print("Row2 = Show 'IP'")
    lcd.text(IP, 2, 'center')
else:
    print("Row2 = ...invalid!")
    
# Leave a logg-text on the console/terminal
# "2019-10-19 Clock=19:59:38.812054 [LinuxStatusOnLCD-display.py] Done"
#print (str(datetime.datetime.now().date()) + " Clock=" + str(datetime.datetime.now().time()) + " [" + sys.argv[0] + "] Done")