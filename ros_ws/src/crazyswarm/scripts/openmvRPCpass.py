#!/usr/bin/env python3
# Remote Control - As The Controller Device
import json, rpc, serial, serial.tools.list_ports, struct, sys
from datetime import datetime
import rospy
import math
from std_msgs.msg import Float64MultiArray
import numpy as np

import csv
command = 0
def getCommand(data):
	global command 
	command = data.data[0]

pub = rospy.Publisher('openmv', Float64MultiArray, queue_size=1)
sub = rospy.Subscriber("openmvCommand", Float64MultiArray, getCommand)
rospy.init_node('talker', anonymous=True)
ave_size = 0

my_array = []
prev_hor = 0
prev_ver = 0

pixel_focal_length = (240) * (2.8) / (3) #
def object_tracking(hor, ver,ave_size):
	# Define camera intrinsic parameters
	#focal_length = 500  # in pixels
	focal_length = 100
	image_width = 160
	image_height = 120
	optical_center_x = image_width / 2
	optical_center_y = image_height / 2
	ver = image_height- ver

	# Define camera extrinsic parameters
	pitch = -.3
	roll = 0
	yaw = .15
	yawMatrix = np.matrix([
	[math.cos(yaw), -math.sin(yaw), 0],
	[math.sin(yaw), math.cos(yaw), 0],
	[0, 0, 1]
	])

	pitchMatrix = np.matrix([
	[math.cos(pitch), 0, math.sin(pitch)],
	[0, 1, 0],
	[-math.sin(pitch), 0, math.cos(pitch)]
	])

	rollMatrix = np.matrix([
	[1, 0, 0],
	[0, math.cos(roll), -math.sin(roll)],
	[0, math.sin(roll), math.cos(roll)]
	])

	R = yawMatrix * pitchMatrix * rollMatrix


	camera_rotation = R  # rotation matrix
	camera_translation = np.array([0, 0, 0])  # translation vector

	# Define object position in image
	object_x = hor  # in pixels
	object_y = ver  # in pixels
	object_width = ave_size  # in pixels
	object_height = ave_size  # in pixels

	# Define actual object size
	actual_width = .25  # in meters
	actual_height = .25  # in meters
	

	# Calculate object center position in image
	object_center_x = object_x + object_width / 2
	object_center_y = object_y + object_height / 2

	# Convert pixel coordinates to normalized image coordinates
	normalized_object_center_x = (object_center_x - optical_center_x) / focal_length
	normalized_object_center_y = (object_center_y - optical_center_y) / focal_length

	# Calculate object distance from camera using known actual distance and object size
	distance_from_camera = actual_width * focal_length / ave_size

	# Calculate object position in 3D space relative to the camera
	object_position_3d = np.array([normalized_object_center_x, normalized_object_center_y, 1])
	object_position_3d = object_position_3d * distance_from_camera

	# Apply camera extrinsic parameters to transform object position into global 3D coordinates
	object_position_3d = np.dot(camera_rotation, object_position_3d)
	object_position_3d = object_position_3d + camera_translation

	# Print object position in 3D space relative to the camera
	print("Object position in 3D space (in meters): ", object_position_3d.round(3)*100)

# color detection remote call
def exe_color_detection(interface):
	global prev_hor
	global prev_ver
	result = interface.call("color_detection")
	if result is not None and len(result):
		res = struct.unpack("<HHHH", result)
		#print("Largest Color Detected: {}".format(res[0]))
		msg = Float64MultiArray()
		# need data in the format [fx,fy, fz, tx, ty, tz] for bicopter we cant do fy or ty, so we will replace with tz or tx
		horizon = round(res[0])
		vertical = round(res[1])
		global ave_size
		if res[2] > 0 or res[3] > 0:
			ave_size = ave_size *.7 + max(res[2], res[3]) * .3
			temp = [round(horizon/160-.5-.15,3), round(vertical/120-.5  -0.016 * ave_size + .425,3), round(ave_size,2), 0]#- (ave_size - 25)/42 
			prev_hor = temp[0]
			prev_ver = temp[1]
			
			my_array.append(temp)
		else:
			temp = [prev_hor,prev_ver,0,0]
			if abs(prev_hor) < .3 and abs(vertical) < .1 and ave_size > 30:
				temp[3] = 1
		#print(temp)

		#if res[2] > 0 or res[3] > 0:
		#		object_tracking(horizon, vertical, ave_size)
		msg.data = temp
		pub.publish(msg)

def exe_light_detection(interface):
	result = interface.call("light_detection")
	if result is not None and len(result):
		res = struct.unpack("<ffff", result)
		#print("Largest Color Detected: {}".format(res[0]))
		msg = Float64MultiArray()
		temp = [res[0], res[1],res[2],res[3]]
		print(temp)
		msg.data = temp
		pub.publish(msg)


if __name__ == "__main__":
	interface = rpc.rpc_network_master(slave_ip="192.168.0.53", my_ip="", port=7610)#0x1DBA) #138
	counter = 0
	while not rospy.is_shutdown():
		counter += 1
		#print(counter)
		sys.stdout.flush()
		#exe_light_detection(interface)
		#interface.call("throw_goal")
		interface.call("throw_blob")
		print(counter)
		#exe_color_detection(interface)
	'''
	with open("data/line.csv", 'w+', newline = "") as csv_file:
		writer = csv.writer(csv_file)
		for row in my_array:
			writer.writerow(row)
			'''
	
			
		
