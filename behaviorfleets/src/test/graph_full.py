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

# Create a single plot with shared y-axis for Freq and Nodes data
fig, ax = plt.subplots(figsize=(10, 5))

# Plot waiting_time_f with freqs as x-axis
ax.plot(freqs, waiting_time_f['mean'], marker='o', label='Frequency', color='blue')

# Fill the area around the line with a shadow representing errors
ax.fill_between(freqs, 
                [m - s for m, s in zip(waiting_time_f['mean'], waiting_time_f['std'])], 
                [m + s for m, s in zip(waiting_time_f['mean'], waiting_time_f['std'])], 
                color='blue', alpha=0.3)

# Create a separate plot for the Nodes data
ax2 = ax.twinx()

# Plot waiting_time_n with nodes as x-axis
ax2.plot(nodes, waiting_time_n['mean'], marker='o', label='Nodes', color='orange')

# Fill the area around the line with a shadow representing errors
ax2.fill_between(nodes, 
                 [m - s for m, s in zip(waiting_time_n['mean'], waiting_time_n['std'])], 
                 [m + s for m, s in zip(waiting_time_n['mean'], waiting_time_n['std'])], 
                 color='orange', alpha=0.3)

# Create a second y-axis for n_upd_f and n_upd_n on the right side
ax3 = ax.twinx()

# Plot n_upd_f with freqs as x-axis on the right y-axis
ax3.plot(freqs, n_upd_f['mean'], marker='s', label='Mean n_upd (Frequency)', color='green')

# Fill the area around the line with a shadow representing errors
ax3.fill_between(freqs, 
                 [m - s for m, s in zip(n_upd_f['mean'], n_upd_f['std'])], 
                 [m + s for m, s in zip(n_upd_f['mean'], n_upd_f['std'])], 
                 color='green', alpha=0.3)

# Create a separate plot for n_upd_n on the right side
ax4 = ax.twinx()

# Plot n_upd_n with nodes as x-axis on the right y-axis
ax4.plot(nodes, n_upd_n['mean'], marker='s', label='Mean n_upd (Nodes)', color='red')

# Fill the area around the line with a shadow representing errors
ax4.fill_between(nodes, 
                 [m - s for m, s in zip(n_upd_n['mean'], n_upd_n['std'])], 
                 [m + s for m, s in zip(n_upd_n['mean'], n_upd_n['std'])], 
                 color='red', alpha=0.3)

# Set labels and title for the x-axis and y-axis
ax.set_xlabel('Frequency - Nodes', fontsize=12)  # Updated x-axis label
ax.set_ylabel('Mean Waiting Time (Frequency)', color='blue')
ax2.set_ylabel('Mean Waiting Time (Nodes)', color='orange')
ax3.set_ylabel('Mean n_upd (Frequency)', color='green')
ax4.set_ylabel('Mean n_upd (Nodes)', color='red')
ax.set_title('Mean Waiting Time vs. Frequency - Nodes')  # Updated title

# Adjust x-axis label positioning and rotation
ax.xaxis.labelpad = 20  # Increase the distance between the label and the axis
ax.xaxis.set_label_coords(0.5, -0.1)  # Adjust label position
ax.tick_params(axis='x', labelrotation=0)  # Rotate x-axis labels if needed

# Set x-axis label for the second plot (Time)
ax2.set_xlabel('Nodes', labelpad=10)  # Set x-axis label for the second plot

# Display the legend
lines, labels = ax.get_legend_handles_labels()
lines2, labels2 = ax2.get_legend_handles_labels()
lines3, labels3 = ax3.get_legend_handles_labels()
lines4, labels4 = ax4.get_legend_handles_labels()
ax.legend(lines + lines2 + lines3 + lines4, labels + labels2 + labels3 + labels4, loc='upper left')

# Customize the appearance of the plot
ax.grid(True, linestyle='--', alpha=0.6)  # Add grid lines with transparency
ax.spines['top'].set_visible(False)  # Remove top and right spines
ax.spines['right'].set_visible(False)
ax.tick_params(axis='both', which='both', length=6, width=1, labelsize=12)  # Customize tick appearance
ax.set_axisbelow(True)  # Place grid lines behind the plot elements

# Create a new figure for Time data
fig2, ax5 = plt.subplots(figsize=(10, 5))

# Plot waiting_time_t with times as x-axis
ax5.plot(times, waiting_time_t['mean'], marker='o', label='Mean Waiting Time (Time)', color='green')

# Fill the area around the line with a shadow representing errors
ax5.fill_between(times, 
                 [m - s for m, s in zip(waiting_time_t['mean'], waiting_time_t['std'])], 
                 [m + s for m, s in zip(waiting_time_t['mean'], waiting_time_t['std'])], 
                 color='green', alpha=0.3)

# Set labels and title for the x-axis and y-axis
ax5.set_xlabel('Time', fontsize=12)
ax5.set_ylabel('Mean Waiting Time (Time)', color='green')
ax5.set_title('Mean Waiting Time vs. Time')

# Create a second y-axis on the right side for n_upd_t
ax6 = ax5.twinx()

# Plot n_upd_t with times as x-axis on the right y-axis
ax6.plot(times, n_upd_t['mean'], marker='s', label='Mean n_upd (Time)', color='blue')

# Fill the area around the line with a shadow representing errors
ax6.fill_between(times, 
                 [m - s for m, s in zip(n_upd_t['mean'], n_upd_t['std'])], 
                 [m + s for m, s in zip(n_upd_t['mean'], n_upd_t['std'])], 
                 color='blue', alpha=0.3)

# Set labels and color for the right y-axis
ax6.set_ylabel('Mean n_upd (Time)', color='blue')

# Display the legend for both lines on the plot
lines5, labels5 = ax5.get_legend_handles_labels()
lines6, labels6 = ax6.get_legend_handles_labels()
ax5.legend(lines5 + lines6, labels5 + labels6, loc='upper left')

# Customize the appearance of the Time plot
ax5.grid(True, linestyle='--', alpha=0.6)
ax5.spines['top'].set_visible(False)
ax5.spines['right'].set_visible(False)
ax5.tick_params(axis='both', which='both', length=6, width=1, labelsize=12)
ax5.set_axisbelow(True)

# Adjust spacing between subplots
plt.subplots_adjust(wspace=0.4)
plt.show(block=False)

while True:
    user_input = input("Press Enter to close the plots...")
    if user_input == "":
        break

# Close the plots
plt.close()
