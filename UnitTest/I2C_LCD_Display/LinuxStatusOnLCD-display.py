# Show Linux/Raspian status/information on OLED-Display (SSD1306)
# Script run once... Supposted to be launched via crontab.
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

# Get local Node/HostName and IP-adress (behind a NAT)
HostName = socket.gethostname()
IP = (([ip for ip in socket.gethostbyname_ex(socket.gethostname())[2] if not ip.startswith("127.")] or [[(s.connect(("8.8.8.8", 53)), s.getsockname()[0], s.close()) for s in [socket.socket(socket.AF_INET, socket.SOCK_DGRAM)]][0][1]]) + ["no IP found"])[0]

lcd = LCD(address=0x3f, width=16, rows=2)
lcd.text(HostName, 1, 'center')
lcd.text(IP,       2, 'center')

print (str(datetime.datetime.now().date()) + " Clock=" + str(datetime.datetime.now().time()) + " [LinuxStatusOnLCD-display] Done")