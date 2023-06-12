from sense_hat import SenseHat
import time

# Leker med den inbyggda 8x8 displayen
# Utgå från hårdkodad bitmap...

sense = SenseHat()

# Definiera/namnge RGB-färger
# ...kolla http://www.colorhexa.com/web-safe-colors
B =  (102, 51,  0)  # Brun "mörkbrun" 
lB=  (205,133, 63)  # Brun "ljusbrun"
b =  (  0,  0,255)  # Blå "kornblå"
lb = (  0,  0,102)  # Blå "ljusblå"
C =  (  0,255,255)  # Cyan/Turkos
W =  (255,255,255)  # Vit
R =  (205,  0,  0)  # Röd
G =  (  0,205,  0)  # Grön
Y =  (255,255,  0)  # Gul
lY=  (255,255,153)  # Gul
Bl=  (  0,  0,  0)  # Svart

# Skapa hårdkodad bitmap...
# ...med följande förlaga: https://amizner.deviantart.com/art/8x8-Pixel-Art-Self-557292245
bitmap8x8 = [
     G, G, B, B, B, B, G, G,
     G, B,lB,lB,lB,lB, G, G,
     G, B,lB, C,lB, C, G, G,
     G,lB,lB,lB,lB,lB, G, G,
     G, R,lB,lB,lB, R, G, G,
    lB, G, R, R, R, R,lB, G,
     G, G,Bl,Bl,Bl, G, G, G,
     G, G,Bl, G,Bl, G, G, G
]

# Visa bitmap på LED matrix
sense.set_pixels(bitmap8x8)

# Fippla lite med ljusstyrka
sense.low_light = False
time.sleep(2)
sense.low_light = True
time.sleep(2)
sense.low_light = False
time.sleep(1)

# Rotera bitmap på LED matrix
sense.set_rotation(90)
time.sleep(1)
sense.set_rotation(180)
