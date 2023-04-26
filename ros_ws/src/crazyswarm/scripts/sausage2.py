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


from std_msgs.msg import  Float64MultiArray
import rospy


class OpenBlimp:
    """Example that connects to a Crazyflie and ramps the motors up/down and
    the disconnects"""

    def __init__(self, swarm):
        """ Initialize and run the example with the specified link_uri """
        '''
        self._cf = Crazyflie(rw_cache='./cache')

        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)
        # self._cf.console.receivedChar.add_callback(self.console_callback)
        self._cf.open_link(link_uri)
        '''
        print("attempting to connect?")
        self.timeHelper = swarm.timeHelper

        self.allcfs = swarm.allcfs

        # for cf in allcfs.crazyflies:
        # self._cf1 = swarm.allcfs.crazyflies[0]
        # self._cf2 = swarm.allcfs.crazyflies[1]
        # self._cf3 = swarm.allcfs.crazyflies[2]
        # self._cf4 = swarm.allcfs.crazyflies[3]
        print("connected")

        
        #print('Connecting to %s' % link_uri)
        
        rospy.Subscriber("/joy", Joy, self.joyChanged)
        print("controller connected")
        

        self.z = None
        self.vz = None
        self.tz = None
        self.pz = None
        self.ptz = None

        self.joy = None

        self.joy_thrust = 0
        self.joy_yaw = 0.
        self.joy_roll = 0.
        self.joy_pitch = 0.

        self.joy_trigger = 0
        self.joy_x = 0
        self.joy_y = 0
        self.joy_a = 0
        self.joy_R = 0
        self.joy_L = 0
        self.b_state = 1
        self.joy_b = 0
        self.cntrl_state = 0 # 0: controller 1: autopilot
        self.x_state = 0
        self.a_state = 0
        self.y_state = 0

        self.joy_pad = 0

        self.joy_dx = 0
        self.joy_dy = 0
        self.joy_tauy = 0
        self.joy_taux = 0
        self.joy_tauz = 0

        

        
        self.z_d = 0

    def _control(self):
        #self._update_params()



        fx = 0#100  # clamped from -1 to 1(for now if we want more max speed we can do more testing)
        fy = 0
        fz = 0#1000 # clamped from 0 to 1.5 # no negative directions yet
        
        tx = 0#.14 # taux has a maximum range from -l to l (l = .15m) before it tries to spin itself
        ty = 0#.25# tauz has a maximum of .25
        tz = 0
        
        height_diff = 0
        rate = 40
        counter = 0
        go_up = 0
 
        print("starting control")

        try:
            
            # Unlock startup thrust protection
            
            start_time = self.timeHelper.time()
            print(start_time)
            while not self.timeHelper.isShutdown():
                self.update_joydata()
                self.timeHelper.sleepForRate(rate)

                
                if self.b_state:
                    self.x_state = 0
                    self.a_state = 0
                    self.y_state = 0
                    fx = 0
                    fz = 0
                    tz = 0
                    tx = 0
                    height_diff = 0
                    
                    self.allcfs.crazyflies[0].cmdFullState([0,0,0], [0,0,0],[15, 0, 0], 0, [0, 0, 0])
                    self.allcfs.crazyflies[1].cmdFullState([0,0,0], [0,0,0],[17, 0, 0], 0, [0, 0, 0])
                    self.allcfs.crazyflies[2].cmdFullState([0,0,0], [0,0,0],[16, 0, 0], 0, [0, 0, 0])
                    self.allcfs.crazyflies[3].cmdFullState([0,0,0], [0,0,0],[18, 0, 0], 0, [0, 0, 0])
                    # self._cf2.cmdFullState([0,0,0], [0,0,0],[6, 0, 0], 0, [0, 0, 0])
                    # self._cf2.cmdFullState([0,0,0], [0,0,0],[5, 0, 0], 0, [0, 0, 0])
                    # self._cf2.cmdFullState([0,0,0], [0,0,0],[6, 0, 0], 0, [0, 0, 0])
                    
                else:
                    '''fx = 0 # range -3.5:3.5 # operation range 0:4
                    fz = .1 #range 0:4 # operation range 0:4
                    tx = 0 # range -.5:.5 # this should at most be .5*fz but even that is too much? 
                            #roll tells us how much power to transfer to one side or the other
                    ty = 0 # range -.5:.5 # yaw is exactly the same thoughts as for roll but on same side
                    tz = .5 # range -.5:5 # this one has much less restrictions since yaw can not cause the motors to go negative when z is positive
                    '''
                    #height_diff = 0
                    #fz = self.joy_pad  #up down
                    height_diff += self.joy_pad /rate

                    fx = -self.joy_dx #forward
                    fy = self.joy_dy
                    tz = self.joy_tauz  #yaw
                    ty = self.joy_tauy #pitch
                    tx = - self.joy_taux     #roll
                    #if self.x_state == 1:
                    #    self.a_state = 0
                    #    fx = .1
                    #    tz = .01
                    if self.joy_a == 1:
                        fz = 0
                        height_diff = 11.2 # 6
                        '''go_up += 1
                        if go_up == 5*rate:
                            if counter%(rate*1) == 0:
                                fx = .2
                            else:
                                fx = -.2 '''
                    elif self.joy_y== 1:
                        fz = 0
                        height_diff = 5
                    else:
                        go_up = 0

                        
                       
                        


                    self.allcfs.crazyflies[0].cmdFullState([fx, fy, fz], [tx, ty, tz],[11, 0, height_diff], 0, [0, 0, 0])
                    self.allcfs.crazyflies[1].cmdFullState([fx, fy, fz], [tx, ty, tz],[13, 0, height_diff], 0, [0, 0, 0])
                    self.allcfs.crazyflies[2].cmdFullState([fx, fy, fz], [tx, ty, tz],[12, 0, height_diff], 0, [0, 0, 0])
                    self.allcfs.crazyflies[3].cmdFullState([fx, fy, fz], [tx, ty, tz],[14, 0, height_diff], 0, [0, 0, 0])
                    # self._cf2.cmdFullState([fx, fy, fz], [tx, ty, tz],[4, 0, 0], 0, [0, 0, 0])
                    # self._cf3.cmdFullState([fx, fy, fz], [tx, ty, tz],[4, 0, 0], 0, [0, 0, 0])
                    # self._cf4.cmdFullState([fx, fy, fz], [tx, ty, tz],[4, 0, 0], 0, [0, 0, 0])
                    
                    


                    if counter%20 == 0:
                        print(fx, fy, fz, tx, ty, tz)
                        counter = 0
                    counter += 1
                
                
                


            
        except Exception as e: 

            exc_type, exc_obj, exc_tb = sys.exc_info()
            fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
            print(exc_type, fname, exc_tb.tb_lineno)       
            
            # Make sure that the last packet leaves before the link is closed
            # since the message queue is not flushed before closing
            self.timeHelper.sleepForRate(rate)
        #self._cf.close_link()

    def joyChanged(self, data):
        global lastData
        lastData = data

    def update_joydata(self):
        if lastData != None:    
            if self.joy_x == 1 and lastData.buttons[2] == 0:
                self.x_state = not self.x_state
                print("afk state", self.x_state)
            if self.joy_b == 1 and lastData.buttons[1] == 0:
                self.b_state = not self.b_state
                print("swap", self.b_state)

            if self.joy_a == 1 and lastData.buttons[0] == 0:
                self.a_state = not self.a_state
                print("up state", self.a_state)
            if self.joy_y == 1 and lastData.buttons[3] == 0:
                self.y_state = not self.y_state
                print("rest flight", self.y_state)

            self.joy_x = lastData.buttons[2]
            self.joy_y = lastData.buttons[3]
            self.joy_a = lastData.buttons[0]
            
            self.joy_b = lastData.buttons[1]
            self.joy_R = lastData.buttons[5] 
            self.joy_L = lastData.buttons[4] 

            self.joy_pad = lastData.axes[1] 
            self.joy_tauz = lastData.axes[0] 
            
            
            self.joy_tauy   = lastData.axes[7] # dz aka height
            self.joy_taux = lastData.axes[6] # taux aka roll

            self.joy_dx   = lastData.axes[4] #dx
            self.joy_dy = lastData.axes[3] #dy
            


            #print(self.joy_thrust, self.joy_pitch, self.joy_roll, self.joy_yaw)

        #     if joy_thrust > 3:
        #         joy_thrust = 3
        #     if joy_thrust < 0:
        #         joy_thrust = 0
        #     else:
        #         joy_yaw=0
        #         joy_pitch=0
        #         joy_roll=0
        #         joy_thrust=0

        # return joy_pitch, joy_roll, joy_thrust, joy_yaw

if __name__ == '__main__':
    # Initialize the low-level drivers
    # cflib.crtp.init_drivers()

    #rospy.init_node("openblimp")

    swarm = Crazyswarm()
    ob = OpenBlimp(swarm)
    ob._control()

    

    
