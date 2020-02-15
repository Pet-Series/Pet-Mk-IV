#!/usr/bin/env python
# -*- coding: iso-8859-15 -*-
# Github @SeniorKullken 2020-01-07
# Github @SeniorKullken 2020-01-04
#
# Show Linux/Raspian status/information on LCD-Display
# Row1: Show local HostName = HostName 
# Row2: Show local IP-address = IP
#
# -----Prerequisite------------------------------
# Display: 1602 16x2 Character LCD Backlight
# I2C interface: PC8574T
#
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
#  ------Future-----------------------------
# TODO Convert "choices=..." to a choices=[choices.keys()]
# TODO Add switch "--nodisplay"
# TODO ADd switch "--text Xyx"
# TODO Plott Graphic data via rqt
# TODO Plott Graphic data 
#      https://stackoverflow.com/questions/7998302/graphing-a-processs-memory-usage
#      https://projects.raspberrypi.org/en/projects/temperature-log/2

from rpi_lcd import LCD
import socket
import datetime
import sys
import subprocess #Utgår? Använd os istället?
import os
import time
import argparse # https://docs.python.org/2/howto/argparse.html

parser = argparse.ArgumentParser(description='Display system status on a LCD-display.',
                                 epilog='Need addional hardware!' )
parser.add_argument("--row1", "-1",type=str, help="What to Display on row #1", default="hostname",
                    choices=["hostname", "CPUload", "RAMusage", "diskusage", "IP", "datetime", "cputemp", "gputemp"])
parser.add_argument("--row2", "-2", type=str, help="What to Display on row #2", default="IP",
                    choices=["hostname", "CPUload", "RAMusage", "diskusage", "IP", "datetime", "cputemp", "gputemp"])
parser.add_argument("--clear", "-c", action="store_true", help="Clear row1 & row2")
parser.add_argument("--verbose", "-v", action="store_true", help="Verbose on the terminal/console")
#parser.add_argument("--nodisplay", "-n", action="store_true", help="I do not have a LCD-dispalay for the moment")
args = parser.parse_args()
#args.verbose = True

#Initiate the SSD1306 LCD-display
lcd = LCD(address=0x3f, width=16, rows=2)

# -------------------------------
# Set/prepare all various parameter to show
#

# Set/Prepare local Node/HostName and IP-adress (behind a NAT). 
host_name=socket.gethostname()

# Set/Prepare CPU-load = CPU
cmd = "top -bn1 | grep load | awk '{printf \"%.2f%%\", $(NF-2)}'"
CPU_load = subprocess.check_output(cmd, shell = True ).decode('utf-8')
    
# Set/Prepare TotaltMemry/UsedMemory information
#cmd = "free -m | awk 'NR==2{printf \"%s/%sMB %.2f%%\", $3,$2,$3*100/$2 }'" #"Total/Used Procent%"
cmd = "free -m | awk 'NR==2{printf \"%s/%sMB %.0f%%\", $3,$2,$3*100/$2 }'" #"Total/Used Procent%"
RAM_usage = subprocess.check_output(cmd, shell = True ).decode('utf-8')
    
# Set/Prepare TotalStorage/UsedStorage information
cmd = "df -h | awk '$NF==\"/\"{printf \"%d/%dGB %s\", $3,$2,$5}'"  #"Total/Used Procent%"
disk_usage = subprocess.check_output(cmd, shell = True ).decode('utf-8')

# Set/Prepare IP-address
# A better algorithm to retrieve IP-address due to complexity with multiple IP and subnet behind NAT.
# IP = (([ip for ip in socket.gethostbyname_ex(socket.gethostname())[2] if not ip.startswith("127.")] or [[(s.connect(("8.8.8.8", 53)), s.getsockname()[0], s.close()) for s in [socket.socket(socket.AF_INET, socket.SOCK_DGRAM)]][0][1]]) + ["no IP found"])[0]
IP = subprocess.check_output(["hostname", "-I"]).decode('utf-8').strip()

# Set/Prepare current date&time 
#date_time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S") # Become tooo long for my 16chr display :-(
date_time = datetime.datetime.now().strftime("%d %b %H:%M:%S")

# Set/Prepare CPU-temperature 
def get_cpu_tempC():
    # First get the CPU/Core temperature
    tempFile = open("/sys/class/thermal/thermal_zone0/temp")
    cpu_temp = tempFile.read()
    tempFile.close()    
    string = "CPU temp.=" + str(round(float(cpu_temp)/1000,1)) + "`C"
    #print (string)
    return string

# Set/Prepare GPU-temperature (GPU=Graphic 'CPU')
def get_gpu_tempC():
    #gpu_temp = subprocess.getoutput( '/opt/vc/bin/vcgencmd measure_temp' ).replace( 'temp=', '' ).replace( '\'C', '' )
    gpu_temp = os.popen('/opt/vc/bin/vcgencmd measure_temp').readline().strip('\n').replace( 'temp=', '' ).replace( '\'C', '' )
    string = "GPU temp.=" + gpu_temp + "`C"
    #print (string)
    return string

if args.verbose:
    verboseHeader = "[" + sys.argv[0] + "]"
    print(verboseHeader + "Verbose mode")

# Is display going to be cleared or not?
if args.clear:
    if args.verbose: print(verboseHeader + "Clear display")
    args.row1 = "customtxt" # Overide/hard-code the row-parameters
    args.row2 = "customtxt" # Overide/hard-code the row-parameters


# Select what to Display on each row
if args.row1 == "hostname":
    if args.verbose: print(verboseHeader + "Row1 = Display 'hostname'=" + host_name)
    lcd.text(host_name, 1, 'center')
elif args.row1 == "CPUload":
    if args.verbose: print(verboseHeader + "Row1 = Display 'CPUload'=" + CPU_load)
    lcd.text(CPU_load, 1, 'right')
elif args.row1 == "RAMusage":
    if args.verbose: print(verboseHeader + "Row1 = Display 'RAMusage'=" + RAM_usage)
    lcd.text(RAM_usage, 1, 'right')
elif args.row1 == "diskusage":
    if args.verbose: print(verboseHeader + "Row1 = Display 'diskusage'=" + disk_usage)
    lcd.text(disk_usage, 1, 'right')
elif args.row1 == "IP":
    if args.verbose: print(verboseHeader + "Row1 = Display 'IP'=" + IP)
    lcd.text(IP, 1, 'center')
elif args.row1 == "datetime":
    if args.verbose: print(verboseHeader + "Row1 = Display 'datetime'=" + date_time)
    lcd.text(date_time, 1, 'left')
elif args.row1 == "cputemp":
    if args.verbose: print(verboseHeader + "Row1 = Display 'cputemp'=" + get_cpu_tempC() )
    lcd.text(get_cpu_tempC(), 1, 'left')
elif args.row1 == "gputemp":
    if args.verbose: print(verboseHeader + "Row1 = Display 'gputemp'=" + get_gpu_tempC() )
    lcd.text(get_gpu_tempC(), 1, 'left')
elif args.row1 == "customtxt":
    if args.verbose: print(verboseHeader + "Row1 = ...clear" )
    lcd.text("---", 1, 'center')
else:
    print(verboseHeader + "Row1 = n/a")


if args.row2 == "hostname":
    if args.verbose: print(verboseHeader + "Row2 = Display 'hostname'=" + host_name)
    lcd.text(host_name, 2, 'center')
elif args.row2 == "CPUload":
    if args.verbose: print(verboseHeader + "Row2 = Display 'CPUload'=" + CPU_load)
    lcd.text(CPU_load, 2, 'right')
elif args.row2 == "RAMusage":
    if args.verbose: print(verboseHeader + "Row2 = Display 'RAMusage'=" + RAM_usage)
    lcd.text(RAM_usage, 2, 'right')
elif args.row2 == "diskusage":
    if args.verbose: print(verboseHeader + "Row2 = Display 'diskusage'=" + disk_usage)
    lcd.text(disk_usage, 2, 'right')
elif args.row2 == "IP":
    if args.verbose: print(verboseHeader + "Row2 = Display 'IP'=" + IP)
    lcd.text(IP, 2, 'center')
elif args.row2 == "datetime":
    if args.verbose: print(verboseHeader + "Row2 = Display 'datetime'=" + date_time)
    lcd.text(date_time, 2, 'left')
elif args.row2 == "cputemp":
    if args.verbose: print(verboseHeader + "Row2 = Display 'cputemp'=" + get_cpu_tempC() )
    lcd.text(get_cpu_tempC(), 2, 'left')
elif args.row2 == "gputemp":
    if args.verbose: print(verboseHeader + "Row2 = Display 'gputemp'=" + get_gpu_tempC() )
    lcd.text(get_gpu_tempC(), 2, 'left')
elif args.row2 == "customtxt":
    if args.verbose: print(verboseHeader + "Row2 = ...clear" )
    lcd.text(" :-) ", 2, 'center')
else:
    print(verboseHeader + "Row2 = n/a")
    
# Leave a logg-text on the console/terminal
# "2019-10-19 Clock=19:59:38.812054 [LinuxStatusOnLCD-display.py] Done"
#print (str(datetime.datetime.now().date()) + " Clock=" + str(datetime.datetime.now().time()) + " [" + sys.argv[0] + "] Done")