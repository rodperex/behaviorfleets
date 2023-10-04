import pandas as pd
import os
import seaborn as sns
import matplotlib.pyplot as plt
import statistics

# Read data from CSV file into a pandas DataFrame
freq_data = pd.read_csv(os.getcwd() + '/freq.csv')

# Define a list of frequencies
freqs = [1, 5, 10, 15, 20, 25, 30, 35, 40, 45]

# Create dictionaries to store coherence, waiting time, success rate, queue size, and mean update values
coherence = {}
waiting_time = {}
success_rate = {}
q_size = {}
n_upd = {}

# Extract 'coherence' column values into the 'coherence' dictionary
aux = freq_data.coherence.tolist()
coherence['mean'] = aux[0::2]  # Extract 'mean' values

# Extract 'mean_wts' column values into the 'waiting_time' dictionary
aux = freq_data.mean_wts.tolist()
waiting_time['mean'] = aux[0::2]  # Extract 'mean' values
waiting_time['std'] = aux[1::2]   # Extract 'std' values

# Extract 'p_succ' column values into the 'success_rate' dictionary
aux = freq_data.p_succ.tolist()
success_rate['mean'] = aux[0::2]  # Extract 'mean' values
success_rate['std'] = aux[1::2]   # Extract 'std' values

# Extract 'q_size' column values into the 'q_size' dictionary
aux = freq_data.max_q.tolist()
q_size['mean'] = aux[0::2]  # Extract 'mean' values
q_size['std'] = aux[1::2]   # Extract 'std' values

# Extract 'mean_nupd' column values into the 'n_upd' dictionary
aux = freq_data.mean_nupd.tolist()
n_upd['mean'] = aux[0::2]  # Extract 'mean' values
n_upd['std'] = aux[1::2]   # Extract 'std' values

print('Averaged success rate: ', statistics.mean(success_rate['mean'] * 10) * 100, '%')

# Create a DataFrame for plotting mean waiting time
data_to_plot_waiting_time = pd.DataFrame({'Freqs': freqs, 'Mean Waiting Time': waiting_time['mean'], 'Std Deviation': waiting_time['std']})

# Create a DataFrame for plotting mean n_upd
data_to_plot_n_upd = pd.DataFrame({'Freqs': freqs, 'Mean n_upd': n_upd['mean'], 'Std Deviation': n_upd['std']})

# Set a professional Seaborn style
sns.set_style("whitegrid")
palette = sns.color_palette("Set1")

# Create a figure and axes for the combined plot (fig1)
fig1, ax1 = plt.subplots(figsize=(10, 6))

# Plot the mean waiting time with a custom color and thicker line width on the left y-axis
sns.lineplot(x="Freqs", y="Mean Waiting Time", data=data_to_plot_waiting_time, marker="o", color=palette[0], label="Mean Waiting Time", ax=ax1, linewidth=2)

# Plot the standard deviation of waiting time as error bars on the left y-axis
ax1.fill_between(data_to_plot_waiting_time["Freqs"], data_to_plot_waiting_time["Mean Waiting Time"] - data_to_plot_waiting_time["Std Deviation"], data_to_plot_waiting_time["Mean Waiting Time"] + data_to_plot_waiting_time["Std Deviation"], alpha=0.3, color=palette[1], label="Std Deviation (Waiting Time)")

# Create a secondary y-axis on the right side for n_upd
ax2 = ax1.twinx()

# Plot the mean n_upd with a custom color and thicker line width on the right y-axis
sns.lineplot(x="Freqs", y="Mean n_upd", data=data_to_plot_n_upd, marker="s", color=palette[2], label="Mean n_upd", ax=ax2, linewidth=2)

# Plot the standard deviation of n_upd as error bars on the right y-axis
ax2.fill_between(data_to_plot_n_upd["Freqs"], data_to_plot_n_upd["Mean n_upd"] - data_to_plot_n_upd["Std Deviation"], data_to_plot_n_upd["Mean n_upd"] + data_to_plot_n_upd["Std Deviation"], alpha=0.3, color=palette[3], label="Std Deviation (n_upd)")

# Set labels and title for the combined plot (fig1)
ax1.set_xlabel('Frequency')
ax1.set_ylabel('Mean Waiting Time', color=palette[0])
ax2.set_ylabel('Mean n_upd', color=palette[2])
plt.title('Mean Waiting Time and Mean n_upd vs. Frequency')

# Relocate the legend for n_upd to the southeast
ax1.legend(loc='upper left')
ax2.legend(loc='lower right')

# Rotate x-axis labels for better readability
plt.xticks(rotation=45)

# Adjust layout for the combined plot (fig1)
plt.tight_layout()

# Create a new figure and axes for the q_size plot (fig2)
fig2, ax_q_size = plt.subplots(figsize=(10, 6))

# Create a DataFrame for plotting mean queue size
data_to_plot_q_size = pd.DataFrame({'Freqs': freqs, 'Mean Queue Size': q_size['mean'], 'Std Deviation': q_size['std']})

# Plot the mean queue size with a custom color and thicker line width
sns.lineplot(x="Freqs", y="Mean Queue Size", data=data_to_plot_q_size, marker="o", color=palette[4], label="Mean Queue Size", ax=ax_q_size, linewidth=2)

# Plot the standard deviation of q_size as error bars
ax_q_size.fill_between(data_to_plot_q_size["Freqs"], data_to_plot_q_size["Mean Queue Size"] - data_to_plot_q_size["Std Deviation"], data_to_plot_q_size["Mean Queue Size"] + data_to_plot_q_size["Std Deviation"], alpha=0.3, color=palette[5], label="Std Deviation (Queue Size)")

# Set labels and title for the q_size plot (fig2)
ax_q_size.set_xlabel('Frequency')
ax_q_size.set_ylabel('Mean Queue Size', color=palette[4])
plt.title('Mean Queue Size vs. Frequency')

# Relocate the legend for n_upd to the southeast
ax_q_size.legend(loc='lower right')

# Rotate x-axis labels for better readability in the q_size plot (fig2)
plt.xticks(rotation=45)

# Adjust layout for the q_size plot (fig2)
plt.tight_layout()

# Show both plots and wait for Enter to be pressed to close them
plt.show(block=False)
while True:
    user_input = input("Press Enter to close the plots...")
    if user_input == "":
        break

# Close the plots
plt.close()
