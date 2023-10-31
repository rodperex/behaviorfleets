import pandas as pd
import os
import seaborn as sns
import matplotlib.pyplot as plt
import statistics

# Read data from CSV file into a pandas DataFrame
freq_data = pd.read_csv(os.getcwd() + '/freq.csv')
nodes_data = pd.read_csv(os.getcwd() + '/nodes.csv')
time_data = pd.read_csv(os.getcwd() + '/time.csv')

# Define a list of frequencies
freqs = [1, 5, 10, 15, 20, 25, 30, 35, 40, 45]
nodes = [2, 5, 10, 15, 20, 25, 30, 35, 40, 45]
times = [1, 2, 4, 8, 16, 32]

# Create dictionaries to store coherence, waiting time, success rate, queue size, and mean update values
coherence_f = {}
waiting_time_f = {}
success_rate_f = {}
q_size_f = {}
n_upd_f = {}
coherence_n = {}
waiting_time_n = {}
success_rate_n = {}
q_size_n = {}
n_upd_n = {}
coherence_t = {}
waiting_time_t = {}
success_rate_t = {}
q_size_t = {}
n_upd_t = {}

# Extract 'coherence' column values into the 'coherence' dictionary
aux = freq_data.coherence.tolist()
coherence_f['mean'] = aux[0::2]  # Extract 'mean' values
aux = nodes_data.coherence.tolist()
coherence_n['mean'] = aux[0::2]  # Extract 'mean' values
aux = time_data.coherence.tolist()
coherence_t['mean'] = aux[0::2]  # Extract 'mean' values

# Extract 'mean_wts' column values into the 'waiting_time' dictionary
aux = freq_data.mean_wts.tolist()
waiting_time_f['mean'] = aux[0::2]  # Extract 'mean' values
waiting_time_f['std'] = aux[1::2]   # Extract 'std' values
aux = nodes_data.mean_wts.tolist()
waiting_time_n['mean'] = aux[0::2]  # Extract 'mean' values
waiting_time_n['std'] = aux[1::2]   # Extract 'std' values
aux = time_data.mean_wts.tolist()
waiting_time_t['mean'] = aux[0::2]  # Extract 'mean' values
waiting_time_t['std'] = aux[1::2]   # Extract 'std' values

# Extract 'p_succ' column values into the 'success_rate' dictionary
aux = freq_data.p_succ.tolist()
success_rate_f['mean'] = aux[0::2]  # Extract 'mean' values
success_rate_f['std'] = aux[1::2]   # Extract 'std' values
aux = nodes_data.p_succ.tolist()
success_rate_n['mean'] = aux[0::2]  # Extract 'mean' values
success_rate_n['std'] = aux[1::2]   # Extract 'std' values
aux = time_data.p_succ.tolist()
success_rate_t['mean'] = aux[0::2]  # Extract 'mean' values
success_rate_t['std'] = aux[1::2]   # Extract 'std' values

# Extract 'q_size' column values into the 'q_size' dictionary
aux = freq_data.max_q.tolist()
q_size_f['mean'] = aux[0::2]  # Extract 'mean' values
q_size_f['std'] = aux[1::2]   # Extract 'std' values
aux = nodes_data.max_q.tolist()
q_size_n['mean'] = aux[0::2]  # Extract 'mean' values
q_size_n['std'] = aux[1::2]   # Extract 'std' values
aux = time_data.max_q.tolist()
q_size_t['mean'] = aux[0::2]  # Extract 'mean' values
q_size_t['std'] = aux[1::2]   # Extract 'std' values

# Extract 'mean_nupd' column values into the 'n_upd' dictionary
aux = freq_data.mean_nupd.tolist()
n_upd_f['mean'] = aux[0::2]  # Extract 'mean' values
n_upd_f['std'] = aux[1::2]   # Extract 'std' values
aux = nodes_data.mean_nupd.tolist()
n_upd_n['mean'] = aux[0::2]  # Extract 'mean' values
n_upd_n['std'] = aux[1::2]   # Extract 'std' values
aux = time_data.mean_nupd.tolist()
n_upd_t['mean'] = aux[0::2]  # Extract 'mean' values
n_upd_t['std'] = aux[1::2]   # Extract 'std' values

# Print averaged success rate
print('Averaged success rate (Frequency): ', statistics.mean(success_rate_f['mean'] * 10) * 100, '%')
print('Averaged success rate (Nodes): ', statistics.mean(success_rate_n['mean'] * 10) * 100, '%')
print('Averaged success rate (Time): ', statistics.mean(success_rate_t['mean'] * 10) * 100, '%')

print('Max. waiting time (Frequency): ', max(waiting_time_f['mean']))
print('Max. waiting time (Nodes): ', max(waiting_time_n['mean']))
print('Max. waiting time (Time): ', max(waiting_time_t['mean']))

# Create a single plot for Mean Waiting Time vs. Frequency, Nodes, and Time
fig, ax1 = plt.subplots(figsize=(10, 5))

# Plot Mean Waiting Time vs. Frequency with shaded error on the primary y-axis
ax1.plot(freqs, waiting_time_f['mean'], marker='o', label='Frequency', color='blue')
ax1.fill_between(freqs,
                 [m - s for m, s in zip(waiting_time_f['mean'], waiting_time_f['std'])],
                 [m + s for m, s in zip(waiting_time_f['mean'], waiting_time_f['std'])],
                 color='blue', alpha=0.3)

# Plot Mean Waiting Time vs. Nodes with shaded error on the primary y-axis
ax1.plot(nodes, waiting_time_n['mean'], marker='o', label='Nodes', color='orange')
ax1.fill_between(nodes,
                 [m - s for m, s in zip(waiting_time_n['mean'], waiting_time_n['std'])],
                 [m + s for m, s in zip(waiting_time_n['mean'], waiting_time_n['std'])],
                 color='orange', alpha=0.3)

# Plot Mean Waiting Time vs. Time with shaded error on the primary y-axis
ax1.plot(times, waiting_time_t['mean'], marker='o', label='Time', color='green')
ax1.fill_between(times,
                 [m - s for m, s in zip(waiting_time_t['mean'], waiting_time_t['std'])],
                 [m + s for m, s in zip(waiting_time_t['mean'], waiting_time_t['std'])],
                 color='green', alpha=0.3)

# Set labels and title for the x-axis and primary y-axis
ax1.set_xlabel('Frequency (Hz) - Nodes (count) - Time (min.)', fontsize=25)
ax1.set_ylabel('Mean Waiting Time (s)', fontsize=25)
# ax1.set_title('Mean Waiting Time vs. Frequency - Nodes - Time', fontsize=25)

# Create a secondary y-axis for n_upd_f, n_upd_n, and n_upd_t
ax2 = ax1.twinx()

# Plot n_upd_f on the secondary y-axis with a thicker line
ax2.plot(freqs, n_upd_f['mean'], marker='x', label='n_upd_f', color='lightblue', linewidth=2.5)

# Plot n_upd_n on the secondary y-axis with darker yellow color
ax2.plot(nodes, n_upd_n['mean'], marker='x', label='n_upd_n', color='darkorange')

# Plot n_upd_t on the secondary y-axis with light green color
ax2.plot(times, n_upd_t['mean'], marker='x', label='n_upd_t', color='lightgreen')

# Set labels for the secondary y-axis
ax2.set_ylabel('Number of Updates', fontsize=18, color='black')

# Display the legend for both primary and secondary y-axes
lines1, labels1 = ax1.get_legend_handles_labels()
lines2, labels2 = ax2.get_legend_handles_labels()
ax1.legend(lines1 + lines2, labels1 + labels2, loc='upper left', fontsize=14)

# Customize the appearance of the plot
ax1.grid(True, linestyle='--', alpha=0.6)
ax1.spines['top'].set_visible(False)
ax1.spines['right'].set_visible(False)
ax1.tick_params(axis='both', which='both', length=6, width=1, labelsize=12)
ax1.set_axisbelow(True)


plt.show(block=False)
while True:
    user_input = input("Press Enter to close the plots...")
    if user_input == "":
        break