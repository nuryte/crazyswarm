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

MVarr = [0,0,0,0,0,0,0]
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
        rospy.Subscriber("/openmv", Float64MultiArray, self.openMV)
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
        self.joy_r = 0
        self.joy_l = 0
        self.joy_line = 0
        self.joy_square = 0
        self.b_state = 1
        self.joy_b = 0
        self.cntrl_state = 0 # 0: controller 1: autopilot
        self.x_state = 0
        self.a_state = 0
        self.y_state = 0
        self.R_state = 0
        self.L_state = 0
        self.r_state = 0
        self.l_state = 0
        self.line_state = 0
        self.square_state = 0


        self.joy_pad = 0

        self.joy_dx = 0
        self.joy_dy = 0
        self.joy_tauy = 0
        self.joy_taux = 0
        self.joy_tauz = 0

        

        
        self.z_d = 0

    
    def openMV(self, data):
        global MVarr
        MVarr[0] = data.data[0]
        MVarr[1] = data.data[1]
        MVarr[2] = data.data[2]
        MVarr[3] = data.data[3]
        

    def _control(self):
        #self._update_params()



        fx = 0#100  # clamped from -1 to 1(for now if we want more max speed we can do more testing)
        fy = 0
        fz = 0#1000 # clamped from 0 to 1.5 # no negative directions yet
        
        tx = 0#.14 # taux has a maximum range from -l to l (l = .15m) before it tries to spin itself
        ty = 0#.25# tauz has a maximum of .25
        tz = 0
        
        fx1 = 0#100  # clamped from -1 to 1(for now if we want more max speed we can do more testing)
        fy1 = 0
        fz1 = 0#1000 # clamped from 0 to 1.5 # no negative directions yet
        
        tx1 = 0#.14 # taux has a maximum range from -l to l (l = .15m) before it tries to spin itself
        ty1 = 0#.25# tauz has a maximum of .25
        tz1 = 0

        fx2 = 0#100  # clamped from -1 to 1(for now if we want more max speed we can do more testing)
        fy2 = 0
        fz2 = 0#1000 # clamped from 0 to 1.5 # no negative directions yet
        
        tx2 = 0#.14 # taux has a maximum range from -l to l (l = .15m) before it tries to spin itself
        ty2 = 0#.25# tauz has a maximum of .25
        tz2 = 0
        height_diff = 0

        height_diff2 = 0


        color_flag = 1
        rate = 40
        counter = 0
        go_up = 0
 
        print("starting control")
        direct_shake = 0
        shake_len = rate*20
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
                    self.R_state = 0
                    self.L_state = 0
                    self.r_state = 0
                    self.l_state = 0
                    self.line_state = 0
                    self.square_state = 0
                    fx = 0
                    fz = 0
                    tz = 0
                    tx = 0
                    height_diff = 0
                    height_diff2 = 0
                    
                    self.allcfs.crazyflies[0].cmdFullState([0,0,0], [0,0,0],[18, 0, 0], 0, [0, 0, 0])
                    self.allcfs.crazyflies[1].cmdFullState([0,0,0], [0,0,0],[16, 0, 0], 0, [0, 0, 0])
                    self.allcfs.crazyflies[2].cmdFullState([0,0,0], [0,0,0],[15, 0, 0], 0, [0, 0, 0])
                    self.allcfs.crazyflies[3].cmdFullState([0,0,0], [0,0,0],[17, 0, 0], 0, [0, 0, 0])

                    self.allcfs.crazyflies[4].cmdFullState([0,0,0], [0,0,0],[18, 0, 0], 0, [0, 0, 0])
                    self.allcfs.crazyflies[5].cmdFullState([0,0,0], [0,0,0],[16, 0, 0], 0, [0, 0, 0])
                    self.allcfs.crazyflies[6].cmdFullState([0,0,0], [0,0,0],[15, 0, 0], 0, [0, 0, 0])
                    self.allcfs.crazyflies[7].cmdFullState([0,0,0], [0,0,0],[17, 0, 0], 0, [0, 0, 0])
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
                    if self.R_state:
                        height_diff += self.joy_pad /rate
                    if self.L_state:
                        height_diff2 += self.joy_pad /rate

                    fx = self.joy_dx * .9#forward
                    fy = self.joy_dy * .9
                    tz = self.joy_tauz  #yaw

                    t_angle = 2.5
                    ty += self.joy_tauy /rate#pitch #
                    if ty > t_angle:
                        ty  = t_angle
                    tx += self.joy_taux /rate    #roll
                    if tx > t_angle:
                        tx  = t_angle
                        


                    if self.joy_x == 1:
                        print("reset angle")
                        ty = 0
                        tx = 0
                        
                    

                    if counter%(shake_len) < (shake_len)/2:
                        direct_shake = .4
                    else:
                        direct_shake = -.4
                    
                        

                    if self.R_state:
                        fx1 = fx#100  # clamped from -1 to 1(for now if we want more max speed we can do more testing)
                        fy1 = fy
                        fz1 = fz#1000 # clamped from 0 to 1.5 # no negative directions yet
                        
                        tx1 = tx#.14 # taux has a maximum range from -l to l (l = .15m) before it tries to spin itself
                        ty1 = ty#.25# tauz has a maximum of .25
                        tz1 = tz
                        if self.a_state == 1:
                            self.line_state = 0
                            fx1 = direct_shake
                            if counter%40 == 0:
                                print("Shake R")
                        elif  self.line_state== 1:
                            ty1 = 2.4
                            fy1 = -.7
                            '''if MVarr[2] != 0:
                                tz1 = -MVarr[0]*5
                                fx1 = .5
                            else:
                                tz1 = .1
                            
                            if counter%40 == 0:
                                print("vision on R")'''
                                

                            #tz = MVarr[1]
                            #fx = MVarr[2]
                    

                    if self.L_state:
                        fx2 = fx#100  # clamped from -1 to 1(for now if we want more max speed we can do more testing)
                        fy2 = fy
                        fz2 = fz#1000 # clamped from 0 to 1.5 # no negative directions yet
                        
                        tx2 = tx#.14 # taux has a maximum range from -l to l (l = .15m) before it tries to spin itself
                        ty2 = ty#.25# tauz has a maximum of .25
                        tz2 = tz
                        if self.y_state == 1:
                            self.square_state = 0
                            fx2 = direct_shake
                            fz2 = 9
                            if counter%40 == 0:
                                print("Shake L")
                        elif self.square_state== 1:
                            #ty2 = 2.4
                            #fy2 = -.7
                            #fz2 = 5
                            height_diff2 = 6.5
                            if MVarr[0] == 1: #== color_flag:
                                tz2 = -MVarr[2]*1 + MVarr[3]*0
                                if fx == 0:
                                    fx2 = .9
                                    
                            else:
                                tz2 = .2
                            
                            if counter%40 == 0:
                                print("vision on L")
                            #tz = MVarr[1]
                            #fx = MVarr[2]
                    
                    if self.r_state:
                        self.allcfs.crazyflies[0].cmdFullState([fx1, fy1, fz1], [tx1, ty1, tz1],[14, 0, height_diff], 0, [0, 0, 0])#8
                        self.allcfs.crazyflies[1].cmdFullState([fx1, fy1, fz1], [tx1, ty1, tz1],[12, 0, height_diff], 0, [0, 0, 0])#2
                        self.allcfs.crazyflies[2].cmdFullState([fx1, fy1, fz1], [tx1, ty1, tz1],[11, 0, height_diff], 0, [0, 0, 0])#4
                        self.allcfs.crazyflies[3].cmdFullState([fx1, fy1, fz1], [tx1, ty1, tz1],[13, 0, height_diff], 0, [0, 0, 0])#6
                    else:
                        height_diff = 0
                        self.line_state = 0
                        self.a_state = 0
                        self.allcfs.crazyflies[0].cmdFullState([0,0,0], [0,0,0],[18, 0, 0], 0, [0, 0, 0])
                        self.allcfs.crazyflies[1].cmdFullState([0,0,0], [0,0,0],[16, 0, 0], 0, [0, 0, 0])
                        self.allcfs.crazyflies[2].cmdFullState([0,0,0], [0,0,0],[15, 0, 0], 0, [0, 0, 0])
                        self.allcfs.crazyflies[3].cmdFullState([0,0,0], [0,0,0],[17, 0, 0], 0, [0, 0, 0])

                    if self.l_state:
                        self.allcfs.crazyflies[4].cmdFullState([fx2, fy2, fz2], [tx2, ty2, tz2],[14, 0, height_diff2], 0, [0, 0, 0])#8
                        self.allcfs.crazyflies[5].cmdFullState([fx2, fy2, fz2], [tx2, ty2, tz2],[12, 0, height_diff2], 0, [0, 0, 0])#2
                        self.allcfs.crazyflies[6].cmdFullState([fx2, fy2, fz2], [tx2, ty2, tz2],[11, 0, height_diff2], 0, [0, 0, 0])#4
                        self.allcfs.crazyflies[7].cmdFullState([fx2, fy2, fz2], [tx2, ty2, tz2],[13, 0, height_diff2], 0, [0, 0, 0])#6
                    else:
                        height_diff2 = 0
                        self.square_state = 0
                        self.y_state = 0
                        self.allcfs.crazyflies[4].cmdFullState([0,0,0], [0,0,0],[18, 0, 0], 0, [0, 0, 0])
                        self.allcfs.crazyflies[5].cmdFullState([0,0,0], [0,0,0],[16, 0, 0], 0, [0, 0, 0])
                        self.allcfs.crazyflies[6].cmdFullState([0,0,0], [0,0,0],[15, 0, 0], 0, [0, 0, 0])
                        self.allcfs.crazyflies[7].cmdFullState([0,0,0], [0,0,0],[17, 0, 0], 0, [0, 0, 0])
                    # self._cf2.cmdFullState([fx, fy, fz], [tx, ty, tz],[4, 0, 0], 0, [0, 0, 0])
                    # self._cf3.cmdFullState([fx, fy, fz], [tx, ty, tz],[4, 0, 0], 0, [0, 0, 0])
                    # self._cf4.cmdFullState([fx, fy, fz], [tx, ty, tz],[4, 0, 0], 0, [0, 0, 0])
                    
                    


                    if counter%40 == 0:
                        print(fx, fy, fz, tx, ty, tz)
                        print(self.L_state, height_diff, ": ",self.R_state, height_diff2)
                        
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
            if self.joy_x == 1 and lastData.buttons[3] == 0:
                self.x_state = not self.x_state
                #print("afk state", self.x_state)
            if self.joy_b == 1 and lastData.buttons[1] == 0:
                self.b_state = not self.b_state
                #print("swap", self.b_state)

            if self.joy_a == 1 and lastData.buttons[0] == 0:
                self.a_state = not self.a_state
                #print("up state", self.a_state)
            if self.joy_y == 1 and lastData.buttons[4] == 0:
                self.y_state = not self.y_state
                #print("rest flight", self.y_state)
            
            if self.joy_R == 1 and lastData.buttons[7] == 0:
                self.R_state = not self.R_state

            if self.joy_L == 1 and lastData.buttons[6] == 0:
                self.L_state = not self.L_state

            if self.joy_r != -1 and lastData.axes[4] == -1:
                self.b_state = 0
                self.r_state = not self.r_state

            if self.joy_l != -1 and lastData.axes[5] == -1:
                self.b_state = 0
                self.l_state = not self.l_state


            if self.joy_line == 1 and lastData.buttons[11] == 0:
                self.line_state = not self.line_state

            if self.joy_square == 1 and lastData.buttons[10] == 0:
                self.square_state = not self.square_state

            self.joy_x = lastData.buttons[3]
            self.joy_y = lastData.buttons[4]
            self.joy_a = lastData.buttons[0]
            
            self.joy_b = lastData.buttons[1]
            self.joy_R = lastData.buttons[7] 
            self.joy_L = lastData.buttons[6] 
            self.joy_line = lastData.buttons[11]
            self.joy_square = lastData.buttons[10]

            self.joy_r = lastData.axes[4] 
            self.joy_l = lastData.axes[5] 

            self.joy_pad = lastData.axes[1] 
            self.joy_tauz = lastData.axes[0] 
            
            
            self.joy_tauy   = lastData.axes[6] # dz aka height
            self.joy_taux = lastData.axes[7] # taux aka roll

            self.joy_dx   = lastData.axes[2] #dx
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

    

    
