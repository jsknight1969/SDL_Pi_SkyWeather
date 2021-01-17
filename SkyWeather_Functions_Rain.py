#Rain functions, variables and calculations

#set config and debug
config = None
SWDEBUG = False

weatherStation = None
rain60Minutes = 0
totalRain = 0

#Rain calculations

rainArray = []
for i in range(60):
	rainArray.append(0)

lastRainReading = 0.0
lastraintime = 0
#lastraincount = weatherStation.get_current_rain_count()

def addRainToArray(plusRain):
	global rainArray
	del rainArray[0]
	rainArray.append(plusRain)
	print("rainArray=", rainArray)

def totalRainArray():
	 global rainArray
	 total = 0
	 for i in range(60):
	 	total = total+rainArray[i]
	 return total
	

def rainRate():
	# currentTR = totalRain
	# global lastraintime
	# currenttime = time.time()
	# rainRateReset()
	# if (lastraintime == 0):
	# 	total = 0
	# else:
	# 	time_delta = (currenttime - lastraintime)
	# 	total = 36000/time_delta * .01
	# 	lastraintime = currenttime
    global lastraincount
    currentcount = weatherStation.get_current_rain_count()
    total = (currentcount - lastraincount) * .01
    print("Last count: %s" % lastraincount)
    lastraincount = currentcount
    print("Current Count: %s" % currentcount)
    print("Rain Rate: %s" % total)
    return total

# def rainRateReset():
# 	global lastraintime
# 	lresetat = lastraintime + 300 
# 	print 'lastraintime: %s' % lastraintime
# 	print 'will update at: %s' % lresetat
# 	print 'current: %s' % time.time()
# 	if (lastraintime > 0) and (lresetat < time.time()):
# 		print ".....resetting rain rate"
# 		lastraintime = 0

def updateRain():
	global lastRainReading, rain60Minutes
	addRainToArray(totalRain - lastRainReading)	
	rain60Minutes = totalRainArray()
	#state.currentRainRate = (totalRain - lastRainReading)*60	
	lastRainReading = totalRain

def statusRain():
        return "Rain in past 60 minutes=%0.2fmm"%rain60Minutes
