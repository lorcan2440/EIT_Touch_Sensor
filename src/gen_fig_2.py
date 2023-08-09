from RectEIT import RectEIT
from utils import *

import numpy as np
import pandas as pd
from matplotlib import pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.font_manager import FontProperties

import time
import threading

stop_event = threading.Event()

def read_eit_voltages(times_list: list, data_list: list, setup: RectEIT,
                      t_start: float, baseline: np.ndarray, electrodes: list[int]) -> np.ndarray:
    while not stop_event.is_set():
        time_elapsed = time.time() - t_start
        relevant_voltages = (setup.read_eit_voltages() - baseline)[list(electrodes)]
        times_list.append(time_elapsed)
        data_list.append(relevant_voltages)

def move_robot(setup: RectEIT, depth: float):
    setup.robot.translatel_rel([0, 0, -0.082, 0, 0, 0], acc=0.05, vel=0.05)
    time.sleep(5)
    setup.robot.translatel_rel([0, 0, -1 * depth, 0, 0, 0], acc=0.05, vel=0.05)
    time.sleep(5)
    setup.robot.translatel_rel([0, 0, depth, 0, 0, 0], acc=0.05, vel=0.05)
    time.sleep(1)
    setup.robot.translatel_rel([0, 0, 0.082, 0, 0, 0], acc=0.05, vel=0.05)
    stop_event.set()

def measure_amplitude(setup: RectEIT, pos: list, depth: float, baseline: np.ndarray,
        electrodes: list[int]) -> tuple[np.ndarray, np.ndarray, list[float]]:  
    '''
    Conduct an EIT experiment at a fixed position and depth.
    Measure relative EIT readings continuously before and during the press (5 seconds each).
    Work out the average relative EIT value up to the press.
    Work out the point of maximum deviation from this average in the period during the press.
    Return the difference between the maximum and the average as the amplitude.
    
    ### Arguments
    #### Required
    - `setup` (RectEIT): setup to use.
    - `pos` (list): frame [x0, x1, y0, y1] to press at. Can use one or two fingers.
    - `depth` (float): depth to press to past the surface.
    - `baseline` (np.ndarray): baseline voltage readings to subtract from.
    - `electrodes` (list[int]): indices of the readings to measure.
    #### Optional
    
    ### Returns
    - `tuple[np.ndarray, np.ndarray, list[float]]`: times list, data list,
    and amplitudes of maximum deviation in voltage during the pressing period.
    '''
    # setup
    vel = [0.1, 0.1, 1.0]
    acc = [0.1, 0.1, 1.0]
    setup.move_up_if_too_low()
    x0, x1, y0, y1 = pos
    num_fingers = 1 if pos.count(None) == 2 else 2

    # get into position
    if num_fingers == 2:
        dist = np.hypot(x0 - x1, y0 - y1)  # distance between fingers
        theta = np.arctan2(x0 - x1, y0 - y1)  # clockwise from positive y-axis
        setup.logger.info(f'Moving to position: '
            f'(x0, x1, y0, y1) = ({round(1000 * x0, 2)}, {round(1000 * x1, 2)}, '
            f'{round(1000 * y0, 2)}, {round(1000 * y1, 2)}); d = {round(1000 * dist, 2)} [mm].')
        x_pivot = x0 - setup.PIVOT_R * np.sin(theta)
        y_pivot = y0 + setup.PIVOT_R * (1 - np.cos(theta))
        target_xyz = setup.frame_xy_to_robot_xy(x_pivot, y_pivot, setup.CORNER_ORIGIN[2])
        setup.robot.translatel(target_xyz, acc=acc[0], vel=vel[0])
        # rotate to position
        setup.robot.movej_rel([0, 0, 0, 0, 0, theta], acc=acc[2], vel=vel[2])
        # extend gear
        setup.move_gear(dist)
    elif num_fingers == 1:
        setup.logger.info(f'Moving to position: '
            f'(x, y) = ({round(1000 * x1, 2)}, {round(1000 * y1, 2)}).')
        setup.move_gear(setup.PIVOT_R, vertical=0)
        target_xyz = setup.frame_xy_to_robot_xy(x1, y1 + setup.PIVOT_R, setup.CORNER_ORIGIN[2])
        setup.robot.translatel(target_xyz, acc=acc[0], vel=vel[0])

    # collect data
    times_list = []
    data_list = []
    t_start = time.time()
    collector_thread = threading.Thread(target=read_eit_voltages,
        args=(times_list, data_list, setup, t_start, baseline, electrodes))
    moving_thread = threading.Thread(target=move_robot, args=(setup, depth))
    moving_thread.start()
    collector_thread.daemon = True
    collector_thread.start()
    moving_thread.join()
    collector_thread.join()
    stop_event.clear()

    # process data
    data_list = np.array(data_list)
    times_list = np.array(times_list)
    avg_voltages = np.mean(data_list[times_list < 5.0], axis=0)  # average the first 5 seconds
    max_deviation = np.max(np.abs(data_list[times_list >= 5.0] - avg_voltages), axis=0)  # maximum deviation from average in data thereafter
    return times_list, data_list, max_deviation


def gen_fig_2(collect_data: bool = True):

    FIG_DATA = 'output/Fig_2_Data.xlsx'

    # set up robot
    setup = RectEIT()
    #setup.connect_to_hardware(move=True, confirm=True)
    #setup.test_corner_calibration_positions()
    #setup.test_gear_motor(num_times=1)

    # set collection sites
    site_names = ['A', 'B', 'C']
    site_A = [None, 0.03, None, 0.05]  # near centre, one finger
    site_B = [None, 0.01, None, 0.05]  # near one edge, one finger
    site_C = [None, 0.01, None, 0.01]  # near corner, one finger
    sites = [site_A, site_B, site_C]
    electrode_names = ['①', '②', '③']
    electrode_1 = electrodes_to_reading_num(4, 6)  # both at top edge
    electrode_2 = electrodes_to_reading_num(1, 5)  # current at top left, voltage at top right
    electrode_3 = electrodes_to_reading_num(21, 13)  # current at bottom edge, voltage at right edge
    electrodes = [electrode_1, electrode_2, electrode_3]
    depth_names = ['4 mm', '8 mm', '12 mm']
    depths = [0.004, 0.008, 0.012]
    selected_num_trials = 5

    t_arrs, v_arrs = [[], []], [[], []]
    if collect_data:
        # gather data
        amplitudes_data = []
        for i, (site_name, site) in enumerate(zip(site_names, sites)):
            for j, (depth_name, depth) in enumerate(zip(depth_names, depths)):
                print(f'Collecting data for site {site_name} at depth {depth_name}: '
                    f'measuring electrodes {electrode_names}')
                is_selected = (i == 1 and j == 2) or (i == 2 and j == 2)
                # if this is one we want to select, repeat several times
                if is_selected:
                    selection = 1 if (i == 1 and j == 2) else 2
                    # selection 1: site B, depth 12 mm, ③. selection 2: site C, depth 12 mm, ②
                    for _trial in range(selected_num_trials):
                        baseline = setup.get_baseline(average=True, num_samples=10)
                        t_arr, v_arr, amplitudes = measure_amplitude(setup, site, depth, baseline, electrodes)
                        t_arrs[selection - 1].append(t_arr)
                        v_arrs[selection - 1].append(v_arr)
                else:  # otherwise, just do it once
                    baseline = setup.get_baseline(average=True, num_samples=10)
                    *_, amplitudes = measure_amplitude(setup, site, depth, baseline, electrodes)
                amplitudes_data.append(amplitudes)  # if selected, the last trial is used
        
        # save amplitude data
        multi_index = pd.MultiIndex.from_product([site_names, depth_names], names=['Press Site', 'Depth'])
        df_amps = pd.DataFrame(amplitudes_data, index=multi_index, columns=electrode_names)
        df_amps.to_excel(FIG_DATA, sheet_name='Amplitudes')

        # save selection time series data
        for selection in [1, 2]:
            dfs = []
            for trial, (t_arr, v_arr) in enumerate(zip(t_arrs[selection - 1], v_arrs[selection - 1]), start=1):
                trial_data = np.concatenate(
                    (np.array(t_arr).reshape((len(t_arr), 1)), np.array(v_arr)), axis=1)
                dfs.append(pd.DataFrame(trial_data, index=None))
            df_all = pd.concat(dfs, axis=1)
            df_all.columns = pd.MultiIndex.from_product(
                [[f'Trial {i}' for i in range(1, selected_num_trials + 1)], 
                ['Time'] + electrode_names], names=["Trial #", "Feature"])
            df_all.to_excel(FIG_DATA, sheet_name=f'Selection {selection}')
    else:
        # read previously collected data
        df_amps = pd.read_excel(FIG_DATA, sheet_name='Amplitudes', index_col=[0, 1])
        t_arrs, v_arrs = [], []
        for selection in [1, 2]:
            df = pd.read_excel(FIG_DATA, sheet_name=f'Selection {selection}', index_col=0, header=[0, 1])
            t_arrs.append(df.loc[:, df.columns.get_level_values(1) == 'Time'].values.T)
            v_arrs.append(np.swapaxes(df.loc[:, df.columns.get_level_values(1) != 'Time'].values.reshape((len(df), -1, 3)), 0, 1))
        t_arrs_1, t_arrs_2 = t_arrs
        v_arrs_1, v_arrs_2 = v_arrs

    # set up axes
    fig = plt.figure(figsize=(10, 5))
    gs = gridspec.GridSpec(2, 6)
    gs.update(wspace=0.5)
    ax_A = plt.subplot(gs[0, 0:2])
    ax_B = plt.subplot(gs[0, 2:4], sharey=ax_A)
    ax_C = plt.subplot(gs[0, 4:6], sharey=ax_A)
    ax_t1 = plt.subplot(gs[1, 1:3])
    ax_t2 = plt.subplot(gs[1, 3:5], sharey=ax_t1)

    # plot top graphs - bar charts
    site_axes = [ax_A, ax_B, ax_C]
    for j, (site_name, site_ax) in enumerate(zip(site_names, site_axes)):
        # put A B C titles
        site_ax.set_title(f'{site_name}', fontweight='bold', size=20)
        # put y-axis label only on first one
        if j == 0:
            site_ax.set_ylabel(r'Peak Response, $ \Delta V_{max} $ (mV)')
        # plot multiple bar charts
        for i, depth_name in enumerate(depth_names):
            div_w = 1 / len(depth_names) * (2/3)
            start_pts = np.arange(3)
            site_ax.bar([n + i * div_w for n in start_pts], 1000 * df_amps.loc[site_name, depth_name].values, width=div_w, label=depth_name)
        # remove spines
        site_ax.spines['top'].set_visible(False)
        site_ax.spines['right'].set_visible(False)
        # put x-axis ticks
        site_ax.legend(loc='upper right')
        site_ax.tick_params(axis='x', which='both', length=0, pad=6)
        site_ax.set_xticks(start_pts + div_w, electrode_names,
            fontproperties=FontProperties(family='DejaVu Sans'), size=20)
    
    # plot bottom graphs - time series
    SELECT_1_ELECTRODE_INDEX = 2  # ③
    SELECT_2_ELECTRODE_INDEX = 1  # ②
    ax_t1.set_title('Response: Site B, 12 mm, ③', 
        fontproperties=FontProperties(family='DejaVu Sans'), fontweight='bold', size=10)
    for trial in range(selected_num_trials):
        ax_t1.plot([t for t in t_arrs_1[trial, :] if t is not None],
            [1000 * v for v in v_arrs_1[trial, :, SELECT_1_ELECTRODE_INDEX] if v is not None],
            alpha=0.5, label=f'Trial {trial + 1}')
    ax_t2.set_title('Response: Site C, 12 mm, ②',
        fontproperties=FontProperties(family='DejaVu Sans'), fontweight='bold', size=10)
    for trial in range(selected_num_trials):
        ax_t2.plot([t for t in t_arrs_2[trial, :] if t is not None], 
            [1000 * v for v in v_arrs_2[trial, :, SELECT_2_ELECTRODE_INDEX] if v is not None],
            alpha=0.5, label=f'Trial {trial + 1}')

    ax_t1.set_xlabel('Time / s')
    ax_t1.set_ylabel(r'Response, $ \Delta V $ (mV)')
    ax_t1.axvspan(7.0, 13.0, alpha=0.3, color='green')
    ax_t2.set_xlabel('Time / s')
    ax_t2.axvspan(7.0, 13.0, alpha=0.3, color='green')
    ax_t1.spines['top'].set_visible(False)
    ax_t1.spines['right'].set_visible(False)
    ax_t2.spines['top'].set_visible(False)
    ax_t2.spines['right'].set_visible(False)

    # show
    plt.subplots_adjust(hspace=0.5)
    plt.tight_layout()
    plt.show()
    plt.savefig('output/Fig_2.svg', format='svg')

if __name__ == '__main__':
    #time_response()
    gen_fig_2(collect_data=False)