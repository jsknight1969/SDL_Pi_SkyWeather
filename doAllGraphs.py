#
# calculate all graphs
#
# SwitchDoc Labs March 30, 2015

import sys
from graphs import TemperatureHumidityGraph, PowerCurrentGraph, PowerVoltageGraph, BarometerLightningGraph
config = None

def doAllGraphs():

	if (config.enable_MySQL_Logging == True):	

		BarometerLightningGraph.BarometerLightningGraph('test', 10, 0)
		TemperatureHumidityGraph.TemperatureHumidityGraph('test', 10, 0)
		PowerCurrentGraph.PowerCurrentGraph('test', 10, 0)
		PowerVoltageGraph.PowerVoltageGraph('test', 10, 0)

