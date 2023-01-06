from rpi_lcd import LCD
from time import sleep

lcd = LCD(address=0x3f, width=16, rows=2)
#lcd.address = 0x3f
lcd.text('Hello World!', 1, 'center')
lcd.text('Raspberry Pi', 2, 'left')


#sleep(5)
#lcd.clear()