#!/usr/bin/env python3
# Remote Control - As The Controller Device
import json, rpc, serial, serial.tools.list_ports, struct, sys
from datetime import datetime
import rospy
import math
from std_msgs.msg import Float64MultiArray
import numpy as np

import csv
command = 1
def getCommand(data):
	global command 
	command = data.data[0]

rospy.Subscriber("openmvCommand", Float64MultiArray, getCommand)
rospy.init_node('openmvCommander', anonymous=True)

if __name__ == "__main__":
	interface = rpc.rpc_network_master(slave_ip="192.168.0.53", my_ip="", port=7610)#0x1DBA) #138
	while not rospy.is_shutdown():
		if command == 0:
			interface.call("throw_blob")
		elif command == 1:
			interface.call("throw_goal")
		elif command == 2:
			interface.call("throw_auto")
		elif command == -1:
			interface.call("throw_end")
			
		
