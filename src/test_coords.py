import pandas as pd
import numpy as np
import openpyxl
from openpyxl import Workbook


servo_angle = 61
# convert servo_angle to a 7-bit binary string
servo_angle_bin = bin(servo_angle)[2:].zfill(7)
# convert vertical to a 1-bit binary string
vertical_bin = bin(1)[2:].zfill(1)
# convert to a 8-bit binary string
# MSB: vertical, 7 LSBs: servo angle (degrees)
data_bin = vertical_bin + servo_angle_bin
# convert to a byte
data = int(data_bin, 2)
data = bytes([data])

print(data)