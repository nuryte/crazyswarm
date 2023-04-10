#!/usr/bin/env python3

import logging
import time
from threading import Thread
import math

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
pastMV = 0
MVarr = [0,0,0, 0]

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
        self._cf = swarm.allcfs.crazyflies[0]
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
        self.past_x = 0
        self.joy_b = 0
        self.cntrl_state = 0 # 0: controller 1: autopilot

        self.joy_pad = 0

        self.joy_dx = 0
        self.joy_dz = 0
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
    def openMV(self, data):
        global MVarr
        MVarr[0] = data.data[0]
        MVarr[1] = data.data[1]
        MVarr[2] = data.data[2]
        MVarr[3] = data.data[3]


    def update_manual(self, fx, fz, tx, tz):
        
        fz += self.joy_dz /rate 
        if fz <0:
            fz = 0
        #if fz > 1.5:
        #    fz = 1.5
        fx = self.joy_dx *.6
        tz = - self.joy_tauz *.15
        tx = - self.joy_taux *-.1#.03
        return fx, fz, tx, tz

    def update_automation(self, fx, fz, tx, tz):
        horizonal = MVarr[0]
        vertical = MVarr[1]
        size = MVarr[2] # in pixels
        if MVarr[3] == pastMV:
            #this means that we dont see anything?
            pass
        fz = vertical
        spread = 15# larger value = smaller spread # 10 is probs minimum 
        mag = 2# increase of the laplace function magnitue
        if size < 200:
            fx = exp(abs(horizontal)*spread)/mag
        else:
            fx = 0
        tz = horizontal * horizontal
        tx = 0
        return fx, fz, tx, tz
        
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
        fz = 0#1000 # clamped from 0 to 1.5 # no negative directions yet

        tx = 0#.14 # taux has a maximum range from -l to l (l = .15m) before it tries to spin itself
        tz = 0#.25# tauz has a maximum of .25
        b_state = False
        x_state = False
        b_old = 0
        motorbase = 30

        horizontal_old = 0
        rate = 40
        counter = 0
 
        print("starting control")
        

        rospy.Subscriber("openmv", Float64MultiArray, self.openMV)

        past = rospy.get_rostime().to_sec()-1


        try:
            
            # Unlock startup thrust protection
            
            start_time = self.timeHelper.time()
            print(start_time)
            while not self.timeHelper.isShutdown():
                
                self.timeHelper.sleepForRate(rate)
                self.update_joydata()
                
                if self.joy_b == 1 and self.past_b == 0:
                    b_state = not b_state
                    x_state = False
                    print("swap", b_state)
                elif self.joy_x == 1 and self.past_x == 0:
                    x_state = not x_state
                    print("automatic swap: ", x_state)



                
                if b_state:
                    fx = 0
                    fz = 0
                    tz = 0
                    tx = 0
                    self._cf.cmdPosition([fx, fz, tz], tx)
                else:
                    if x_state:
                        
                        horizontal = MVarr[0]
                        vertical = MVarr[1]
                        size = MVarr[2] # in pixels
                        if size == 0:
                            fx = 0
                            tx = 0
                            tz = 0
                        else:
                            
                            #fz += vertical
                            spread = 30# larger value = smaller spread # 10 is probs minimum 
                            mag = .01# increase of the laplace function magnitue
                            if size < 2000:
                                if abs(horizontal) < .15:
                                    fx = .1 
                            else:
                                fx = 0
                            
                            tz = -horizontal* 300000 /(size)#- (horizontal_old-horizontal)/rate*5
                            if tz > .5:
                                tz = .5
                            horizontal_old = horizontal
                            tx = 0
                        
                    else:
                        
                        fz += self.joy_dz /rate 
                        if fz <0:
                            fz = 0
                        #if fz > 1.5:
                        #    fz = 1.5
                        fx = self.joy_dx 
                        tz = - self.joy_tauz *.6
                        tx = - self.joy_taux *-.3#.03
                    self._cf.cmdVelocityWorld([fx, fz, tz], tx)


                #tx = 0
                #fz = 1
                #tauz = 1
                #taux = 0
                if counter%20 == 0:
                    print(fx, fz, tz, tx)
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
            self.past_x = self.joy_x 
            self.joy_x = lastData.buttons[2]
            self.joy_y = lastData.buttons[3]
            self.joy_a = lastData.buttons[0]
            self.past_b = self.joy_b 
            self.joy_b = lastData.buttons[1]
            self.joy_R = lastData.buttons[5] 
            self.joy_L = lastData.buttons[4] 

            self.joy_pad = lastData.buttons[7] 
            
            
            self.joy_dz   = lastData.axes[1] # dz aka height
            self.joy_taux = lastData.axes[2] # taux aka roll

            self.joy_dx   = lastData.axes[3] #dx
            self.joy_tauz = lastData.axes[0] #tauz
            


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

    

    
