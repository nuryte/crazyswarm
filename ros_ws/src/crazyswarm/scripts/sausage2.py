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
        self._cf1 = swarm.allcfs.crazyflies[0]
        self._cf2 = swarm.allcfs.crazyflies[1]
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
        self.past_b = 0
        self.joy_b = 0
        self.cntrl_state = 0 # 0: controller 1: autopilot

        self.joy_pad = 0

        self.joy_dx = 0
        self.joy_dy = 0
        self.joy_tauy = 0
        self.joy_taux = 0
        self.joy_tauz = 0

        
        self.z_d = 0

    '''
    def _set_param(self, cf, groupstr, namestr, value):
        """Function to set parameters in the firmware of crazyflie specified

        Args:
            cf (crazyflie): Crazyflie to have its parameters changed
            groupstr (string): Parameter group
            namestr (string): Parameter name
            value (float): New parameter value
        """
        full_name = groupstr+"."+namestr
        self._cf.param.add_update_callback(group=groupstr, name = namestr, cb = self._param_callback)
        self._cf.param.set_value(full_name, value)

    def _param_callback(self, name, value):
        """Callback for crazyflie parameter data

        Args:
            name (string): name of parameter
            value (int): value of parameter
        """
        print('The crazyflie has parameter ' + name + ' set at number: ' + value)

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""

        # Start a separate thread to do the motor test.
        # Do not hijack the calling thread!
        print("Connected!")
        Thread(target=self._control).start()

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)

    def _update_params(self):
        self._set_param(self._cf, 'stabilizer', 'controller', 2) #Set estimator to Melinger controller (2)#Kalman Filter
        #self._set_param(self._cf, 'stabilizer', 'estimator', 1) #Set estimator to Kalman Filter
        self._set_param(self._cf, 'pid_attitude', 'pitch_kd', 2.) #Default: 0.1
        self._set_param(self._cf, 'pid_attitude', 'pitch_ki', 0) #.1Default: 3
        self._set_param(self._cf, 'pid_attitude', 'pitch_kp', 5.) #Default: 0.1

        self._set_param(self._cf, 'pid_attitude', 'roll_kd', 1.) #Default: 0.1
        self._set_param(self._cf, 'pid_attitude', 'roll_ki', 0.) #Default: 3
        self._set_param(self._cf, 'pid_attitude', 'roll_kp', 5.) #Default: 0.1

        self._set_param(self._cf, 'pid_attitude', 'yaw_kd', 0.35) #Default: 0.1
        self._set_param(self._cf, 'pid_attitude', 'yaw_ki', 0.) #Default: 1
        self._set_param(self._cf, 'pid_attitude', 'yaw_kp', 6.) #Default: 0.1

        self._set_param(self._cf, 'pid_rate', 'pitch_kd', 5) #2.5#Default: 0.1
        self._set_param(self._cf, 'pid_rate', 'pitch_ki', 0.) #Default: 500
        self._set_param(self._cf, 'pid_rate', 'pitch_kp', 250.) #Default: 0.1

        self._set_param(self._cf, 'pid_rate', 'roll_kd', 2.5) #Default: 0.1
        self._set_param(self._cf, 'pid_rate', 'roll_ki', 0.) #Default: 500
        self._set_param(self._cf, 'pid_rate', 'roll_kp', 250.) #Default: 0.1

        self._set_param(self._cf, 'pid_rate', 'yaw_kd', 0.1) #0#Default: 0.1
        self._set_param(self._cf, 'pid_rate', 'yaw_ki', 0) #Default: 16.7
        self._set_param(self._cf, 'pid_rate', 'yaw_kp', 60.) #60#Default: 120
        pass

    '''
    def _control(self):
        #self._update_params()

        #s1 and s2 are servos
        #m1 and m2 are motors
        #m values are out of 100%
        maxima = 65000
        mult = 650
        minimum = 0

        s1 = 0        
        s2 = 0

        m1 = 0
        m2 = 0

        fx = 0#100  # clamped from -1 to 1(for now if we want more max speed we can do more testing)
        fy = 0
        fz = 0#1000 # clamped from 0 to 1.5 # no negative directions yet
        
        tx = 0#.14 # taux has a maximum range from -l to l (l = .15m) before it tries to spin itself
        ty = 0#.25# tauz has a maximum of .25
        tz = 0
        b_state = True
        b_old = 0
        motorbase = 30

        rate = 40
        counter = 0
 
        print("starting control")

        try:
            
            # Unlock startup thrust protection
            
            start_time = self.timeHelper.time()
            print(start_time)
            while not self.timeHelper.isShutdown():
                
                self.update_joydata()
                self.timeHelper.sleepForRate(rate)

                if self.joy_b == 1 and self.past_b == 0:
                    b_state = not b_state
                    print("swap", b_state)
                
                if b_state:
                    fx = 0
                    fz = 0
                    tauz = 0
                    taux = 0
                    self._cf1.cmdFullState([0,0,0], [0,0,0],[5, 0, 0], 0, [0, 0, 0])
                    self._cf2.cmdFullState([0,0,0], [0,0,0],[6, 0, 0], 0, [0, 0, 0])
                    
                else:
                    '''fx = 0 # range -3.5:3.5 # operation range 0:4
                    fz = .1 #range 0:4 # operation range 0:4
                    tx = 0 # range -.5:.5 # this should at most be .5*fz but even that is too much? 
                            #roll tells us how much power to transfer to one side or the other
                    ty = 0 # range -.5:.5 # yaw is exactly the same thoughts as for roll but on same side
                    tz = .5 # range -.5:5 # this one has much less restrictions since yaw can not cause the motors to go negative when z is positive
                    '''
                    fz = self.joy_pad  #up down
                    
                    #if fz > 1.5:
                    #    fz = 1.5
                    fx = self.joy_dx #forward
                    fy = self.joy_dy
                    tz = self.joy_tauz  #yaw
                    ty = self.joy_tauy #pitch
                    tx = - self.joy_taux     #roll
                    self._cf1.cmdFullState([fx, fy, fz], [tx, ty, tz],[3, 0, 0], 0, [0, 0, 0])
                    self._cf2.cmdFullState([fx, fy, fz], [tx, ty, tz],[4, 0, 0], 0, [0, 0, 0])
                    
                    


                #tx = 0
                #fz = 1
                #tauz = 1
                #taux = 0
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
            self.joy_x = lastData.buttons[2]
            self.joy_y = lastData.buttons[3]
            self.joy_a = lastData.buttons[0]
            self.past_b = self.joy_b 
            self.joy_b = lastData.buttons[1]
            self.joy_R = lastData.buttons[5] 
            self.joy_L = lastData.buttons[4] 

            self.joy_pad = lastData.axes[1] 
            self.joy_tauz = lastData.axes[0] 
            
            
            self.joy_tauy   = lastData.axes[7] # dz aka height
            self.joy_taux = lastData.axes[6] # taux aka roll

            self.joy_dx   = lastData.axes[3] #dx
            self.joy_dy = lastData.axes[2] #dy
            


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

    

    
