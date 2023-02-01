# Show Linux/Raspian status/information on OLED-Display (SSD1306)
# Script run once... Supposted to be launched via crontab.
#
# Get local HostName = HostName 
# Get locval IP-adfress = IP
# Get CPU-load = CPU
# Get TotaltMemry/UsedMemory info. = MemUsage
# Get TotalStorage/UsedStorage info. = Disk
# 
import time
import datetime

import Adafruit_GPIO.SPI as SPI
import Adafruit_SSD1306

from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont

import subprocess
import socket

# Raspberry Pi pin configuration:
RST = None     # on the PiOLED this pin isnt used
# Note the following are only used with SPI:
DC = 23
SPI_PORT = 0
SPI_DEVICE = 0

# 128x32 display with hardware I2C:
#disp = Adafruit_SSD1306.SSD1306_128_32(rst=RST)

# 128x64 display with hardware I2C:
disp = Adafruit_SSD1306.SSD1306_128_64(rst=RST, i2c_bus=1)

# Initialize library.
disp.begin()

# Clear display.
disp.clear()
disp.display()

# Create blank image for drawing.
# Make sure to create image with mode '1' for 1-bit color.
width = disp.width
height = disp.height
image = Image.new('1', (width, height))
draw = ImageDraw.Draw(image)

# Draw a black filled box to clear the image.
#draw.rectangle((0,0,width,height), outline=0, fill=0)

# First define some constants to allow easy resizing of shapes.
padding = -2
top = padding
bottom = height-padding

# Move left to right keeping track of the current x position for drawing shapes.
x = 0

# Load default font.
font = ImageFont.load_default()

# Get local Node/HostName
HostName = socket.gethostname()

# Get locval IP-adfress behind a NAT)
IP = (([ip for ip in socket.gethostbyname_ex(socket.gethostname())[2] if not ip.startswith("127.")] or [[(s.connect(("8.8.8.8", 53)), s.getsockname()[0], s.close()) for s in [socket.socket(socket.AF_INET, socket.SOCK_DGRAM)]][0][1]]) + ["no IP found"])[0]

# Get CPU-load = CPU
cmd = "top -bn1 | grep load | awk '{printf \"%.2f%%\", $(NF-2)}'"
CPU = subprocess.check_output(cmd, shell = True ).decode('utf-8')
    
#Get TotaltMemry/UsedMemory information
cmd = "free -m | awk 'NR==2{printf \"%s/%sMB %.2f%%\", $3,$2,$3*100/$2 }'" #"Total/Used Procent%"
MemUsage = subprocess.check_output(cmd, shell = True ).decode('utf-8')
    
#Get TotalStorage/UsedStorage information
cmd = "df -h | awk '$NF==\"/\"{printf \"%d/%dGB %s\", $3,$2,$5}'"  #"Total/Used Procent%"
Disk = subprocess.check_output(cmd, shell = True ).decode('utf-8')

# Write some lines of text.
draw.text((x, top),    "Host: " + str(HostName),font=font, fill=255)
draw.text((x, top+9),  "IP: "  + str(IP),       font=font, fill=255)
draw.text((x, top+30), "CPU: " + str(CPU),      font=font, fill=255)
draw.text((x, top+40), "RAM: " + str(MemUsage), font=font, fill=255)
draw.text((x, top+50), "Disk:" + str(Disk),     font=font, fill=255)

# Display image.
disp.image(image)
disp.display()
print (str(datetime.datetime.now().date()) + " Clock=" + str(datetime.datetime.now().time()) + " [LinuxStatusOnOLED-display] Done")