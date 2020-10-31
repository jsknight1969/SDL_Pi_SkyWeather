try:
	import conflocal as config
except ImportError:
	import config


import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(config.rainPin, GPIO.IN)
GPIO.add_event_detect(config.rainPin, GPIO.RISING, callback=serviceInterruptRain, bouncetime=40  )  
_tick = False


def serviceInterruptRain(self,channel):
		
		#print "Rain Interrupt Service Routine"
        if (_tick):
  		    print("tick")
        else:
            print("tock")
        _tick = not _tick

try:
    print "waiting on rain"

except KeyboardInterrupt:
    GPIO.cleanup()
GPIO.cleanup()
	



