import sys
import time

sys.path.append('./SDL_Pi_SSD1306')
sys.path.append('./Adafruit_Python_SSD1306')

import Adafruit_SSD1306
import Scroll_SSD1306

RST =27
display = Adafruit_SSD1306.SSD1306_128_64(rst=RST, i2c_address=0x3C)
# Initialize library.
display.begin()
display.clear()
display.display()

Scroll_SSD1306.addLineOLED(display,  "TESTING OLED")
