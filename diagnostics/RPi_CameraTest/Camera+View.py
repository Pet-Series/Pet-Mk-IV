import picamera
from time import sleep
import os
# Xlib:  extension "RANDR" missing on display ":10.0".
#(gpicview:2869):
# GLib-GObject-WARNING **:
# Attempt to add property GtkSettings:
# :gtk-scrolled-window-placement after class was initialised

camera = picamera.PiCamera()
camera.rotation = 180
print ('klick1.py: Take picture')
camera.capture('python-camera.jpg')
print ('klick1.py: Launch Viewer')
os.system('gpicview python-camera.jpg &amp;')
print ('klick1.py: Wait 1')
sleep(2)
print ('klick1.py: Wait 2')
sleep(2)
print ('klick1.py: Wait 3')
sleep(2)
print ('klick1.py: Close')
os.system('killall gpicview')
#camera.start_preview()
#sleep(5)
#camera.stop_preview()
