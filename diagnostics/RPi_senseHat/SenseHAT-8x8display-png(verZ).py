from sense_hat import SenseHat
import time
# Leker med den inbyggda 8x8 displayen
# Utgå från inlästa .png filer...
sense = SenseHat()

# Istället för att hårdkodad bitmap...
# ...läs in från .PNG-fil istället
# Bit map nr.1
bitmap8x8 = sense.load_image("pilUpp.png", redraw=False)

# Visa bitmap på LED matrix
sense.set_pixels(bitmap8x8)

# Rotera bitmap på LED matrix
time.sleep(1)
sense.set_rotation(90)
time.sleep(1)
sense.set_rotation(180)
time.sleep(1)

# Bit map nr.2
bitmap8x8 = sense.load_image("äpple.png", redraw=False)
sense.set_pixels(bitmap8x8)
time.sleep(2)

# Bit map nr.3
bitmap8x8 = sense.load_image("krokodil.png", redraw=False)
sense.set_pixels(bitmap8x8)
time.sleep(2)

# Bit map nr.4
bitmap8x8 = sense.load_image("anka.png", redraw=False)
sense.set_pixels(bitmap8x8)
