from RectEIT import RectEIT
import numpy as np
import time
import threading
from matplotlib import pyplot as plt

stop_event = threading.Event()

global t_press_down
global t_press_up


def read_eit_voltages(times_list: list, data_list: list, setup: RectEIT,
                      t_start: float, baseline: np.ndarray, electrodes: list[int]) -> np.ndarray:
    while not stop_event.is_set():
        res = time.time() - t_start, (setup.read_eit_voltages(check_full=False) - baseline)#[list(electrodes)]
        times_list.append(res[0])
        data_list.append(res[1])

def move_robot(setup: RectEIT, depth: float):
    setup.robot.translatel_rel([0, 0, -0.082, 0, 0, 0], acc=0.05, vel=0.05)
    time.sleep(5)
    t_press_down = time.time()
    setup.robot.translatel_rel([0, 0, -1 * depth, 0, 0, 0], acc=0.05, vel=0.05)
    time.sleep(5)
    setup.robot.translatel_rel([0, 0, depth, 0, 0, 0], acc=0.05, vel=0.05)
    time.sleep(5)
    setup.robot.translatel_rel([0, 0, 0.082, 0, 0, 0], acc=0.05, vel=0.05)
    stop_event.set()

def gen_fig_2():
    setup = RectEIT()
    setup.connect_to_hardware(move=False)
    setup.move_up_if_too_low()

    baseline = setup.get_baseline(average=True, num_trials=20, check_full=False)
    #baseline_avg = np.mean(baseline, axis=0)

    n_rows, n_cols = 6, 6
    fig, axs = plt.subplots(n_rows, n_cols, figsize=(12, 8), sharex=True, sharey=True)

    for j, depth in enumerate([0, 0.004, 0.008, 0.012]):
        # get into position
        setup.move_up_if_too_low()
        pos = [0.04, 0.04, 0.09, 0.03]
        x0, x1, y0, y1 = pos
        dist = 0.06
        theta = 0.0
        x_pivot = x0 - self.PIVOT_R * np.sin(theta)
        y_pivot = y0 + self.PIVOT_R * (1 - np.cos(theta))
        z_curr = setup.robot.getl()[2]
        target_xyz = setup.frame_xy_to_robot_xy(x_pivot, y_pivot, z_curr)
        setup.robot.translatel(target_xyz, acc=0.1, vel=0.1)
        setup.move_gear(dist, vertical=1)

        electrodes = np.linspace(0, 1023, 36).astype(int)  # [116, 146, 204, 223, 438, 613, 847, 876]

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

        # data_list = [[V(t=0, el=0), V(t=0, el=1), ...], [V(t=1, el=0), V(t=1, el=1), ...], ...]
        for i, ax in enumerate(axs.flat, start=0):
            ax.plot(times_list, np.array(data_list)[:, i], label=f'{depth * 1000} mm')
            ax.set_xlabel('Time / s')
            ax.set_ylabel('Response / V')
            if j == 0:  # first only
                ax.set_title(f"Reading #{electrodes[i]}")
                ax.axvspan(7.0, 13.0, alpha=0.3, color='green')
            elif j == 3:  # last only
                ax.legend(loc='upper right')
        
        stop_event.clear()

    fig.suptitle('Responses of selected electrodes to a step strain')
    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    gen_fig_2()    