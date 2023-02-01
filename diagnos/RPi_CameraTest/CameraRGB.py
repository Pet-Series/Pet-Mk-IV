import picamera
import picamera.array
##import matplotlib.pyplot as plt
import time
import os
# Xlib:  extension "RANDR" missing on display ":10.0".
#(gpicview:2869):
# GLib-GObject-WARNING **:
# Attempt to add property GtkSettings:
# :gtk-scrolled-window-placement after class was initialised

camera = picamera.PiCamera()
camera.rotation = 180
camera.resolution = (1280, 720)

output = picamera.array.PiRGBArray(camera)

camera.capture(output, 'rgb') #Saves the array

print ("Done")
print (output.array.shape)

##plt.imshow(output.array.array)
##print (output.array(1,1,1))

##print('Captured %dx%d image' % (
##        output.array.shape[1], output.array.shape[0]))
##print (len(a))


#camera.start_preview()
#time.sleep(5)
#camera.stop_preview()
