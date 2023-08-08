from RectEIT import RectEIT
import numpy as np


def main():

    # setup equipment
    ross = RectEIT()
    ross.connect_to_hardware(move=True, confirm=True)
    ross.test_corner_calibration_positions()
    ross.test_gear_motor(num_times=2)

    # get baseline
    baseline = ross.get_baseline(output_file=RectEIT._OUTPUT_BASELINE, check_full=False)
    baseline_avg = np.mean(baseline, axis=0)
    ross.show_voltage_graph(baseline)

    #pos, vol, _ = ross.get_voltages_at_rect_pos(0.05, None, 0.02, None, num_trials=3, baseline=baseline_avg, average=False)
    #ross.show_voltage_graph(vol, baseline=baseline_avg)

    ross.get_randomised_data(num_datapoints=300, baseline=True, take_baseline_every=10)

    #pos = ross.get_random_pos()
    #_pos, pred_pos = ross.touch_prediction(*pos, ..., num_trials=1, show_results=True)
    #img = ross.convert_xy_to_disp_map(pred_pos)
    #ross.show_disp_map(img)

    #pos = [0.05, 0.03, 0.02, 0.06]
    #img = ross.convert_xy_to_disp_map(pos)
    #ross.show_disp_map(img)

    ross.disconnect_from_hardware()


if __name__ == '__main__':
    main()