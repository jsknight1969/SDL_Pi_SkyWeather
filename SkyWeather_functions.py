import socket
import Scroll_SSD1306
import sys
import sendemail


from Adafruit_Python_SSD1306.Adafruit_SSD1306 import SSD1306
from Adafruit_Python_GPIO.Adafruit_GPIO import GPIO, SPI

sys.path.append('./Adafruit_Python_GPIO')

global display
display = None
RST = 27
SWDEBUG = False
config = None
toAddress = None
fromAddress = None
txtAddress = None

def get_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    IP = '127.0.0.1'
    try:
        s.connect(('10.255.255.255', 8888))
        IP = s.getsockname()[0]
    except Exception as e:
        print(e)
    finally:
        if (SWDEBUG): print("IP: %s" % IP)
        s.close()
    return IP

def initializeOLED():
    global display 
    _return = False
    try:
        # Initialize library.
        RST = 27
        display = SSD1306.SSD1306_128_64(rst=RST, i2c_address=0x3C)
        display.begin()
        display.clear()
        display.display()
        Scroll_SSD1306.addLineOLED (display, "Starting SkyWeather")
        Scroll_SSD1306.addLineOLED (display, get_ip())
        _return = True
    except Exception as e:
        #handle it
        display = None
        if (SWDEBUG): print(e)

    finally:
        return _return

def writetoOLED(text):
    Scroll_SSD1306.addLineOLED(display, text)

def displayLightningAlert():
    writetoOLED('')
    writetoOLED('--- LIGHTNING ---')
    writetoOLED('')

def returnStatusLine(device, state):

        returnString = device
        returnString = "%20s" % device
        statetext = "Not Present"
        if (state): statetext = "Present"
        returnString = returnString + (" - %10s" % statetext)
        return returnString

def sendEmail(bodyText, subjectText):
    #sendemail.sendEmail("test", bodyText, subjectText ,config.notifyAddress,  config.fromAddress, "")
    sendemail.sendEmail("test", bodyText, subjectText , toAddress,  fromAddress, "")

def sendText(bodyText, subjectText):
    #sendemail.sendEmail("test", bodyText, subjectText ,config.notifyAddress,  config.fromAddress, "")
    sendemail.sendEmail("test", bodyText, subjectText , txtAddress,  fromAddress, "")

