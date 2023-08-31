# local imports
import kg_robot as kgr
from utils import ColouredLoggingFormatter

# third party imports
import numpy as np
import pandas as pd
import serial
import openpyxl
from matplotlib import pyplot as plt

# built ins
import logging
import os
import subprocess
import datetime as dt
import random
import time
import math
import pickle

# robot coordinate system:
# x: forward / backward
# y: left / right
# z: up / down

# home position - origin
#home = np.array([-0.576529, -0.314895, 0.22, -3.13687, 0.0739108, 0.0466963])
home = np.array([-0.551269, -0.218117, 0.18000, 2.91132, 1.15728, -0.0453532])

# corners
#_corner_x = np.array([-0.500932, -0.320676, 0.22, -3.13634, -0.0449873, 0.0429553])
#_corner_y = np.array([-0.579264, -0.200348, 0.22, -3.13622, -0.0450577, 0.0462504])

# claw tool centre point
claw_tcp = [-0.055398, 0, 0.06382, 0, 0, 0]


####################

if __name__ == '__main__':

    robot = kgr.kg_robot(**{'port': 30010, 'db_host': '169.254.5.1'})
    time.sleep(0.5)
    robot.set_tcp(claw_tcp)

    print(robot.getl())
    print(robot.getj())

    #robot.movel(home, acc=0.05, vel=0.05)


    #robot.movel([-0.539573, -0.220754, 0.248809, -3.14124, 0, 0], acc=0.05, vel=0.05)
    #robot.movej([0.21835, -1.00886, 1.14622, -4.85013, 1.5692, 4.93086], acc=0.05, vel=0.05)

    #robot.movel_tool([0, 0, 0, 0, 0, 0.5], acc=0.05, vel=0.05)  # 

    #print(robot.getl(), robot.getj())

    #robot.movej_rel([0, 0, 0, 0, 0, 1], acc=0.05, vel=0.05)
    #robot.translatel_rel([0.1, 0.1, 0, 0, 0, 0], acc=0.05, vel=0.05)
    #robot.movej_rel([0, 0, 0, 0, 0, -1], acc=0.05)

    # 0.75:     5.74813
    # 0.5:      5.49813
    # 0:        4.99813  -  this is dependent on position!!
    # -2.4:     2.59813


    #print(robot.getl(), robot.getj())

    robot.close()




