from RectEIT import RectEIT
import numpy as np


def main():
    # setup equipment
    ross = RectEIT()
    ross._OUTPUT_FILE = 'output/EIT_Data_Gelatin_6_fingers - SD - demo set.xlsx'
    ross.connect_to_hardware(move=True, confirm=True)
    ross.test_six_finger_motor(num_times=3)
    ross.get_random_six_finger_data(num_trials=1000)


if __name__ == '__main__':
    main()
