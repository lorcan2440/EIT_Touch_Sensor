import pandas as pd
import numpy as np
from matplotlib import pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.font_manager import FontProperties
import matplotlib.patches as patches

site_names = ['A', 'B', 'C']
electrode_names = ['①', '②', '③']
depth_names = ['4 mm', '8 mm', '12 mm']
selected_num_trials = 5

amplitudes_data = np.random.random((9, 3))
amp_mindex = pd.MultiIndex.from_product([site_names, depth_names], names=['Press Site', 'Depth'])
df_amps = pd.DataFrame(amplitudes_data, index=amp_mindex, columns=electrode_names)

dt_arrs_1 = np.random.uniform(0, 1.0, size=(selected_num_trials, 30))
dt_arrs_2 = np.random.uniform(0, 1.0, size=(selected_num_trials, 35))
t_arrs_1 = np.cumsum(dt_arrs_1, axis=1)
t_arrs_2 = np.cumsum(dt_arrs_2, axis=1)
v_arrs_1 = np.random.random((selected_num_trials, 30, 3))
v_arrs_2 = np.random.random((selected_num_trials, 35, 3))

def test():
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
    test()