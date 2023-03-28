#!/usr/bin/env python

import numpy as np

from pycrazyswarm import *
import uav_trajectory



if __name__ == "__main__":
    print("STARTING UP")
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    print("attempting to connect?")
    cf = swarm.allcfs.crazyflies[0]
    print("connected")

    rate = 10.0
    
    start_time = timeHelper.time()
    print("GOING IN")
    while not timeHelper.isShutdown():
    

        cf.cmdVelocityWorld([0,0,0],0)
        #cf.cmdPosition([0,0,0],0)

        timeHelper.sleepForRate(rate)
    print("GOING DARK")

