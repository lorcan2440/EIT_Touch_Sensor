import numpy as np
import pandas as pd
import openpyxl
import sys
sys.path.append('src')

from RectEIT import RectEIT

r = RectEIT()

pos = r.get_random_pos(num_fingers=1)

print(pos)