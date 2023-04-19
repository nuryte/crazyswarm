#!/usr/bin/env python3

import logging
import time
from threading import Thread
import os, sys

#import cflib
#from cflib.crazyflie import Crazyflie
#from cflib.utils import uri_helper
from sensor_msgs.msg import Joy

from pycrazyswarm import *


#uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E702')#radio://0/100/2M/E7E7E7E708   'usb://0')# 

logging.basicConfig(level=logging.ERROR)

lastData = None
joy_pitch = 0
joy_roll = 0
joy_thrust = 0
joy_yaw = 0
MVarr = [0,0,0,0]

from std_msgs.msg import  Float64MultiArray
import rospy


class OpenBlimp:
    """Example that connects to a Crazyflie and ramps the motors up/down and
    the disconnects"""

    def __init__(self, swarm, blimpType = "mochi"):
        
        print("attempting to connect")
        self.timeHelper = swarm.timeHelper
        if blimpType == "mochi":
            self._cf1 = swarm.allcfs.crazyflies[0]
        else:#blimpType == "sausage":
            self._cf1 = swarm.allcfs.crazyflies[0]
            self._cf2 = swarm.allcfs.crazyflies[1]
        print("cf connected ", blimpType)
        
        rospy.Subscriber("/joy", Joy, self.joyChanged)
        self.joyOpenmv = rospy.Publisher("/openmvCommand", Float64MultiArray, queue_size = 1)
        print("controller connected")
        
        self.joy = None

        self.joy_trigger = 0
        self.joy_x = 0
        self.joy_y = 0
        self.joy_a = 0
        self.joy_R = 0
        self.joy_L = 0
        self.joy_b = 0
        
        self.joy_pad = 0
        self.joy_dx = 0
        self.joy_dy = 0
        self.joy_tauy = 0
        self.joy_taux = 0
        self.joy_tauz = 0

        self.b_state = True

        self.x_state = False

        self.y_state = 1

    def _control(self):

        fx = 0#100  # clamped from -1 to 1(for now if we want more max speed we can do more testing)
        fy = 0
        fz = 0#1000 # clamped from 0 to 1.5 # no negative directions yet
        
        tx = 0#.14 # taux has a maximum range from -l to l (l = .15m) before it tries to spin itself
        ty = 0#.25# tauz has a maximum of .25
        tz = 0
        

        rate = 40
        counter = 0
        horizontal_old = 0
        vertical_old = 0
        size_old = 0
        capture_timer = 3*rate
        print("starting control")
        

        if True:#try:
            
            # Unlock startup thrust protection
            
            start_time = self.timeHelper.time()
            print(start_time)
            while not self.timeHelper.isShutdown():
                
                self.update_joydata()
                self.timeHelper.sleepForRate(rate)


                if self.b_state:
                    fx = 0
                    fz = 0
                    tauz = 0
                    taux = 0
                    capture_timer = 3*rate
                    x_state = False
                    self._cf1.cmdFullState([0,0,0], [0,0,0],[7, 0, 0], 0, [0, 0, 0])
                elif self.x_state:
                    if self.y_state == 1:
                        msg = Float64MultiArray()
                        msg.data = [1]
                        self.joyOpenmv.publish(msg)
                    elif self.y_state == 0:
                        msg = Float64MultiArray()
                        msg.data = [0]
                        self.joyOpenmv.publish(msg)

                    fx = 0
                    fz = 0
                    tauz = 0
                    taux = 0
                    self._cf1.cmdFullState([0,0,0], [0,0,0],[0, 1, 0], 0, [0, 0, 0])
                    
                    
                    
                else:
                    fz = self.joy_pad  #up down
                    fx = self.joy_dx  #forward
                    fy = self.joy_dy
                    tz = - self.joy_tauz *1.5 #yaw
                    ty = - self.joy_tauy *.5 #pitch
                    tx = self.joy_dy*.5#- self.joy_taux *.5 #roll
                    self._cf1.cmdFullState([fx, fy, fz], [tx, ty, tz],[0, 0, 0], 0, [0, 0, 0])
                    
                    
                    


                #tx = 0
                #fz = 1
                #tauz = 1
                #taux = 0
                if counter%20 == 0:
                    print(fx, fy, fz, tx, ty, tz)
                    counter = 0
                counter += 1
                
                
                


        '''   
        except Exception as e: 

            exc_type, exc_obj, exc_tb = sys.exc_info()
            fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
            print(exc_type, fname, exc_tb.tb_lineno)       
            
            # Make sure that the last packet leaves before the link is closed
            # since the message queue is not flushed before closing
            self.timeHelper.sleepForRate(rate)
        #self._cf.close_link()'''

    def joyChanged(self, data):
        global lastData
        lastData = data

    def update_joydata(self):
        if lastData != None:
            if self.joy_b == 0 and lastData.buttons[1] == 1:
                self.b_state = not self.b_state
                print("swap: ", self.b_state)
            if self.joy_x == 0 and lastData.buttons[3] == 1:
                self.x_state = not self.x_state
                print("automatic swap: ", self.x_state)
            if self.joy_y == 0 and lastData.buttons[4] == 1:
                if self.y_state == 0:
                    self.y_state = 1
                    print("ball detection: ", self.y_state)
                if self.y_state == 1:
                    self.y_state = 0
                    print("goal detection: ", self.y_state)
                
                
            self.joy_x = lastData.buttons[3]
            self.joy_y = lastData.buttons[4]
            self.joy_a = lastData.buttons[0]
            self.joy_b = lastData.buttons[1]
            #self.joy_R = lastData.buttons[5] 
            #self.joy_L = lastData.buttons[4] 

            self.joy_pad = lastData.axes[1] 
            self.joy_tauz = lastData.axes[0] 
            
            
            self.joy_tauy   = lastData.axes[6] # dz aka height
            self.joy_taux = lastData.axes[7] # taux aka roll

            self.joy_dx   = lastData.axes[3] #dx
            self.joy_dy = lastData.axes[2] #dy
            


if __name__ == '__main__':
    # Initialize the low-level drivers
    # cflib.crtp.init_drivers()
    n = len(sys.argv)
    blimpType = "mochi"
    if n > 1:
        blimpType = sys.argv[1]

    swarm = Crazyswarm()
    ob = OpenBlimp(swarm, blimpType)
    ob._control()

    

    
