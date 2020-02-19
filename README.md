SkyWeather Libraries and Examples for Raspberry Pi Solar Powered Weather Station<BR>

Supports SwitchDoc Labs WeatherRack PiWeather Board <BR> 

All documentation is on:<BR>

https://shop.switchdoc.com/products/skyweather-raspberry-pi-based-weather-station-kit-for-the-cloud

December 15, 2019: Version 055 - MySQL SolarMAX Fixes


http://www.switchdoc.com/<BR>


November 27, 2019: Version 054 - Fixed reporting of SolarMAX inside temperature/humidity
November 26, 2019: Version 053 - Update Blynk with latest SolarMAX Packet Status
November 2, 2019: Version 052 - Fixed WeatherUnderground URL and Added more debug for LoRa WXLink 
October 15, 2019: Version 051 - Added support for SolarMAX Lead Acid - Must update conflocal.py if used<BR>
<pre>
@@ -42,6 +42,10 @@ runLEDs = False
 SolarMAX_Present = False
 Dual_MAX_WXLink = False
 
+# SolarMAX_Type = "LEAD" for SolarMAX Lead Acid
+# SolarMAX_Type = "LIPO" for SolarMAX LiPo
+SolarMAX_Type = ""
+

</pre><BR>
October 14, 2019: Version 050 - Fixed Camera Detection on Buster<BR>
October 12, 2019:   Version 049 - Fixed BlynkBug / AM2315 Bug <BR>
September 29, 2019: Version 048 - Fixed SolarMAX bug <BR>
September 1, 2019: Version 047 - Fixed to Camera Exposure, Minor tweak to WeatherSTEM Interface and SolarMAX (added Version)<BR>
August 19, 2019: Version 046 - Minor Bug release (matplotlib, SolarMAX, blynk)<BR>
August 14, 2019: Version 045 - Camera Debug Support - SolarMAX support - Must update conflocal.py<BR>
August 12, 2019: Version 044 - Camera Debug Support - Overexposure problem<BR>
August 8, 2019:  Version 043 - Improved AM2315 Detection, SQL Structure Fixed, time and date changed, debug for overexposure <BR>
August 6, 2019:  Version 042 - Overlays, Lightning Params added - Must update conflocal.py if used<BR>
July 27, 2019:   Version 041 - Fix to SHT30 for > 122 degrees <BR>
July 8, 2019:    Version 040 - WeatherUnderground Fix, Support for SHT30- Must update conflocal.py if used<BR>
June 5, 2019:    Version 039 - AM2315 Reliablity Fix <BR>
May 21, 2019:    Version 038 - Blynk Bug Fix<BR>
May 21, 2019:    Version 037 - Blynk Changes / Bug Fix<BR>
May 20, 2019:    Version 036 - Fixed Barometric Pressure Reporting<BR>
May 12, 2019:    Version 035 - Debug Statements removed<BR>
May 4, 2019:     Version 034 - WeatherSTEM testing Version<BR>
May 1, 2019:     Version 033 - WeatherSTEM API Started<BR>
April 29, 2019:  Version 033 - WeatherSTEM Modification<BR>
April 28, 2019:  Version 032 - Improved MySQL Reporting<BR>
April 28, 2019:  Version 031 - Fixed WXLink Temperature Reporting<BR>
April 27, 2019:  Version 030 - Modified test programs<BR>
April 20, 2019:  Version 029 - Fixed Lightning_Mode added Image test to blynkCode
April 6, 2019:   Version 028 - Support for WXLink - remote WeatherRack/Temp/Humidity
April 3, 2019:   Version 027 - Mod AS3935 Interrupt, added AQI to Database<BR>
March 31, 2019:  Version 026 - Fixed Pins for Optional Fan On/Off<BR>

-----------------
Updating conflocal.py on your System<BR>
-----------------

Run this command:

diff conflocal.py config.py

Add the new config.py variables into your conflocal.py version for compatiblity


-----------------<BR>

Install this for smbus:

sudo apt-get install python-smbus

Install this next:


git clone https://github.com/adafruit/Adafruit_Python_PureIO.git<BR>
cd Adafruit_Python_PureIO<BR>
sudo python setup.py install<BR>

Other installations required for AM2315:

sudo apt-get install python-pip

sudo apt-get install libi2c-dev


#Installing apscheduler

sudo pip install --upgrade setuptools pip <BR>

sudo pip install setuptools --upgrade  <BR>
sudo pip install apscheduler <BR>

#Installing pigiod

pigpiod is used to get accurate timing readings for the Air Quality sensor. <BR>

sudo apt-get install pigpio

#installing matplotlib

sudo apt-get install python-numpy python-matplotlib python-mpltoolkits.basemap 



----------------<BR>
Note some configurations of Raspberry Pi software requres the following:<BR>
It won't hurt to do this in any case.<BR>
----------------<BR>
<pre>
sudo apt-get update
sudo apt-get install build-essential python-pip python-dev python-smbus git
git clone https://github.com/adafruit/Adafruit_Python_GPIO.git
cd Adafruit_Python_GPIO
sudo python setup.py install
cd ..
cd SDL_Pi_SkyWeather
cd Adafruit_Python_SSD1306
sudo python setup.py install
</pre>
SwitchDocLabs Documentation for WeatherRack/WeatherPiArduino under products on: store.switchdoc.com

Read the SkyWeather Instructable on instructables.com for more software installation instructions 

or

Read the tutorial on SkyWeather on http://www.switchdoc.com/
for more software installation instructions.

-----------
setup your configuration variables in config.py!
-----------
We recommend you copy config.py to conflocal.py to avoid updates copying over your configuration file.<BR>
--------
Add SQL instructions
----------

Use phpmyadmin or sql command lines to add the included SQL file to your MySQL databases.<BR>
Note:  If the database has been updated, run the example below to update your database.   The current contents will not be lost.

To install mysql ( https://www.stewright.me/2016/04/install-mysql-server-raspberry-pi/ ) <BR>

also run this for the Python to MySQL bindings:

sudo apt-get install python-mysqldb


cd SkyWeatherSQL

sudo mysql -u root -p < WeatherPiStructure.sql

user:  root

password: password

Obviously with these credentials, don't connect port 3306 to the Internet.   Change them if you aren't sure.

The phpmyadmin user: admin <BR>
password:   password <BR>

NOTE:

If you have a WXLink wireless transmitter installed, the software assumes you have connected your AM2315 outdoor temp/humidity sensor to the WXLink.  If you put another AM2315 on your local system, it will use those values instead of the WXLink values

-------------------<BR>
# Starting the SkyWeather.py program
-------------------<BR>

You start the program with two statements:

sudo pigpiod
sudo python SkyWeather.py

-------------------<BR>
Set up your rc.local for start on boot<BR>
-------------------<BR>

insert the following in your /etc/rc.local before the exit 0 statement:

pigpiod
cd /home/pi/SDL_Pi_SkyWeather <BR>
nohup sudo python SkyWeather.py & <BR>


--------------------<BR>  

 # Run skyweather.py as a service  
 Thanks to user WBP on forum.switchdoc.com for these instructions  
 Read more: http://forum.switchdoc.com/thread/1110/running-skyweather-boot-another-approach#ixzz6DzQHQbhJ

-------------------<BR>  
 I've always used a script in /etc/init.d for things that need to be run at boot, rather than making changes to /etc/rc.boot.  There are some advantages to doing it this way.  For example, you can enter this command to start SkyWeather:  
<pre>
    sudo /etc/init.d/skyweather start  
</pre>
and this command to stop it:  
<pre>
    sudo /etc/init.d/skyweather stop  
</pre>  

In my startup script I also gave the output file a name with the date in it.  This way there is a new file each time SkyWeather is started, and you can easily purge the older ones.  You *really* do NOT want to let nohup.out get so big it fills up the file system - it can be a real pain dealing with a Linux system where the file system is completely full.

Here's how to do this:  

1) It's easier to do this if you modify SkyWeather.py so it can be run without typing "python".  Insert this as the first line:  
<pre>
    #!/usr/bin/python  
</pre>
2) Then make it executable:  
<pre>
    sudo chmod +x SkyWeather.py  
</pre>  
3) Now create a new script in /etc/init.d  
<pre>
    sudo nano /etc/init.d/skyweather
</pre>
4) Copy and paste this into nano editor
<pre>
    #!/bin/sh
    # /etc/init.d/skyweather
    
    ### BEGIN INIT INFO  
    # Provides: SkyWeather  
    # Required-Start: $local_fs $remote_fs $syslog $time  
    # Required-Stop: $local_fs $remote_fs $syslog  
    # Default-Start: 2 3 4 5  
    # Default-Stop: 0 1 6  
    # Short-Description: start SkyWeather from boot  
    # Description: A simple script which will start a program from boot and stop upon shut-down  
    ### END INIT INFO  

    # Put any commands you always want to run here.  

    TIME=`date +"%y%m%d.%H%M"` # get a date/timestamp    
    DATE=`date +"%y%m%d"` # get a datestamp  
    echo $TIME  
    case "$1" in  
        start)  
            echo "Starting SkyWeather"  
            #run the program you want to start  
            cd /home/pi/SDL_Pi_SkyWeather  
            pigpiod  
            ./SkyWeather.py >> skyweather.$DATE.log &  
            chgrp pi skyweather.$DATE.log  
        ;;  
        stop)  
            echo "Stopping SkyWeather"  
            #end the program you want to stop  
            killall "SkyWeather.py"  
        ;;  
    *)  
    echo "Usage: /etc/init.d/skyweather {start|stop}"  
    exit 1  
    ;;  
    esac  
</pre>

5) Save the nano file with CTRL+X and follow the prompts

6) Make the script executable:  
<pre>
    sudo chmod +x /etc/init.d/skyweather  
</pre>
7) Tell the init system about it:  
<pre>
    sudo update-rc.d skyweather defaults  
</pre>

That's it!  Don't forget to remove the changes from /etc/rc.boot.  



------------<BR>

This works great from my personal experience.  May make updates a bit tougher but I think the trade offs are worth it.

