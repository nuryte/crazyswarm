#!/usr/bin/env python3
# Remote Control - As The Controller Device
import json, rpc, serial, serial.tools.list_ports, struct, sys, rospy
from std_msgs.msg import Float64MultiArray
from datetime import datetime

# color detection remote call
def exe_color_detection(interface):
    result = interface.call("difference_goal_blob")
    if result is not None and len(result):
        res = struct.unpack("<HHH", result)
        #print("Largest Color Detected: cx={}, cy={}, size={}".format(res[0], res[1], res[2]))
        msg = Float64MultiArray()
        # need data in the format [fx,fy, fz, tx, ty, tz] for bicopter we cant do fy or ty, so we will replace with tz or tx
        horizon = (round(res[0]/160,3)-0.5)
        vertical = (round(res[1]/120,3)-0.5)
        temp = [horizon, vertical, res[2]]
        msg.data = temp
        pub.publish(msg)

if __name__ == "__main__":
    pub = rospy.Publisher('openmv', Float64MultiArray, queue_size=1)
    rospy.init_node('talker', anonymous=True)
    interface = rpc.rpc_network_master(slave_ip="192.168.0.42", my_ip="", port=0x1DBA)
    counter = 0
    #rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        sys.stdout.flush()
        exe_color_detection(interface)
        #rate.sleep()