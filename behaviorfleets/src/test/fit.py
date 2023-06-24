import os
import math
import pandas as pd
import numpy as np
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt
import seaborn as sns
from sklearn.linear_model import LinearRegression


def func(x, a, b):
  return a / np.sqrt(x) + b

def parabola(x, a, b, c):
  return a*x*x/b + c

def logarithmic(x, a, b):
    return (a * np.log(x*x*x*x)) + b


def plot_results_nodes(x, avg_wt, max_wt, avg_succ, max_succ, min_succ, tam_q):
  fig, axes = plt.subplots(nrows=2, ncols=2)
  fig.suptitle('NUMBER OF NODES')
  
  # params, _ = curve_fit(parabola, x, avg_wt)
  # a, b, c = params
  # y1 =  parabola(x, a, b, c)
  coefficients = np.polyfit(x, avg_wt, 2)
  p = np.poly1d(coefficients)
  y1 = p(x)
  axes[0, 0].plot(x, avg_wt, 'b')
  axes[0, 0].plot(x, max_wt, 'g')
  axes[0, 0].plot(x, y1, 'r')
  axes[0, 0].grid(True)
  axes[0, 0].set_xlabel('Number of nodes')
  axes[0, 0].set_ylabel('Waiting time')
  axes[0, 0].legend(['Average', 'Maximum', 'Curve fit'])


  model = LinearRegression().fit(x.reshape(-1, 1), avg_succ)
  y1 = model.predict(x.reshape(-1, 1))
  axes[0, 1].plot(x, avg_succ, 'b')
  axes[0, 1].plot(x, y1, 'r')
  axes[0, 1].plot(x, max_succ, 'g')
  axes[0, 1].plot(x, min_succ, 'y')
  axes[0, 1].grid(True)
  axes[0, 1].set_xlabel('Number of nodes')
  axes[0, 1].set_ylabel('Success rate')
  axes[0, 1].legend(['Average', 'Linear fitd', 'Maximum', 'Minimum'])


  [a, b]= np.polyfit(x, tam_q, 1)
  y1 = a * x + b
  axes[1, 0].plot(x, tam_q, 'b')
  axes[1, 0].plot(x, y1, 'r')
  axes[1, 0].grid(True)
  axes[1, 0].set_xlabel('Number of nodes')
  axes[1, 0].set_ylabel('Queue size')
  axes[1, 0].legend(['Maximum queue size', 'Linear fit'])

  fig.delaxes(axes[1, 1])
  plt.tight_layout()
  plt.show(block=False)


def plot_results_time(x, avg_wt, max_wt, avg_succ, max_succ, min_succ, tam_q):

  sns.set(style="darkgrid")  # Set the style of the plots to darkgrid

  # Create the figure and subplots using Seaborn's subplots function
  fig, axes = plt.subplots(nrows=2, ncols=2, figsize=(10, 8))

  # Plot the waiting time
  sns.lineplot(x=x, y=avg_wt, ax=axes[0, 0], color='b')
  # sns.lineplot(x=x, y=max_wt, ax=axes[0, 0], color='g')
  # axes[0, 0].legend(['Average', 'Maximum'])
  
  # Fit and plot the linear regression line for waiting time
  sns.regplot(x=x, y=avg_wt, ax=axes[0, 0], color='r', scatter=False)

  # Set the labels and legends for waiting time subplot
  axes[0, 0].set_xlabel('Operation time')
  axes[0, 0].set_ylabel('Waiting time')
  

  # Plot the success rate
  sns.lineplot(x=x, y=avg_succ, ax=axes[0, 1], color='b')
  
  # Fit and plot the linear regression line for success rate
  sns.regplot(x=x, y=avg_succ, ax=axes[0, 1], color='r', scatter=False)

  # Plot the maximum and minimum success rate
  # sns.lineplot(x=x, y=max_succ, ax=axes[0, 1], color='g')
  # sns.lineplot(x=x, y=min_succ, ax=axes[0, 1], color='y')
  
  # Set the labels and legends for success rate subplot
  axes[0, 1].set_xlabel('Operation time')
  axes[0, 1].set_ylabel('Success rate')
  axes[0, 1].legend(['Average', 'Linear fit', 'Maximum', 'Minimum'])

  # Plot the queue size
  sns.lineplot(x=x, y=tam_q, ax=axes[1, 0], color='b')
  
  # Fit and plot the linear regression line for queue size
  sns.regplot(x=x, y=tam_q, ax=axes[1, 0], color='r', scatter=False)

  # Set the labels and legends for queue size subplot
  axes[1, 0].set_xlabel('Operation time')
  axes[1, 0].set_ylabel('Queue size')
  axes[1, 0].legend(['Maximum queue size', 'Linear fit'])

  # Remove the empty subplot
  fig.delaxes(axes[1, 1])

  # Adjust the spacing between subplots
  plt.tight_layout()
  
  # Show the plot
  plt.show()


def plot_results_time2(x, avg_wt, max_wt, avg_succ, max_succ, min_succ, tam_q):
  nan_indices = np.isnan(avg_wt)
  avg_wt = avg_wt[~nan_indices]
  max_wt = max_wt[~nan_indices]
  avg_succ = avg_succ[~nan_indices]
  max_succ = max_succ[~nan_indices]
  min_succ = min_succ[~nan_indices]
  tam_q = tam_q[~nan_indices]
  # x = x[~nan_indices]

  fig, axes = plt.subplots(nrows=2, ncols=2)
  fig.suptitle('OPERATION TIME')

  [a, b] = np.polyfit(x, avg_wt, 1)
  y1 = a * x + b
  axes[0, 0].plot(x, avg_wt, 'b')
  axes[0, 0].plot(x, max_wt, 'g')
  axes[0, 0].plot(x, y1, 'r')
  axes[0, 0].grid(True)
  axes[0, 0].set_xlabel('Operation time')
  axes[0, 0].set_ylabel('Waiting time')
  axes[0, 0].legend(['Average', 'Maximum', 'Linear fit'])


  [a, b] = np.polyfit(x, avg_succ, 1)
  y1 = a * x + b
  # params, _ = curve_fit(logarithmic, x, avg_succ)
  # a, b = params
  # y1 =  logarithmic(x, a, b)
  axes[0, 1].plot(x, avg_succ, 'b')
  axes[0, 1].plot(x, y1, 'r')
  axes[0, 1].plot(x, max_succ, 'g')
  axes[0, 1].plot(x, min_succ, 'y')
  axes[0, 1].grid(True)
  axes[0, 1].set_xlabel('Operation time')
  axes[0, 1].set_ylabel('Success rate')
  axes[0, 1].legend(['Average', 'Linear fit', 'Maximum', 'Minimum'])
  
  [a, b] = np.polyfit(x, tam_q, 1)
  y1 = a * x + b
  axes[1, 0].plot(x, tam_q, 'b')
  axes[1, 0].plot(x, y1, 'r')
  axes[1, 0].grid(True)
  axes[1, 0].set_xlabel('Operation time')
  axes[1, 0].set_ylabel('Queue size')
  axes[1, 0].legend(['Maximum queue size', 'Linear fit'])

  fig.delaxes(axes[1, 1])
  plt.tight_layout()
  plt.show(block=False)


def plot_results_freq(x, avg_wt, max_wt, avg_succ, max_succ, min_succ, tam_q):
  fig, axes = plt.subplots(nrows=2, ncols=2)
  fig.suptitle('STRESSER FREQUENCY')

  [a, b] = np.polyfit(x, avg_wt, 1)
  y1 = a * x + b
  axes[0, 0].plot(x, avg_wt, 'b')
  axes[0, 0].plot(x, max_wt, 'g')
  axes[0, 0].plot(x, y1, 'r')
  axes[0, 0].grid(True)
  axes[0, 0].set_xlabel('Stresser Hz')
  axes[0, 0].set_ylabel('Waiting time')
  axes[0, 0].legend(['Average', 'Maximum', 'Linear fit'])
  # axes[0, 0].legend(['Average', 'Maximum'])
  

  [a, b] = np.polyfit(x, avg_succ, 1)
  y1 = a * x + b
  axes[0, 1].plot(x, avg_succ, 'b')
  axes[0, 1].plot(x, y1, 'r')
  axes[0, 1].plot(x, max_succ, 'g')
  axes[0, 1].plot(x, min_succ, 'y')
  axes[0, 1].grid(True)
  axes[0, 1].set_xlabel('Stresser Hz')
  axes[0, 1].set_ylabel('Success rate')
  axes[0, 1].legend(['Average', 'Linear fit', 'Maximum', 'Minimum'])
  # axes[0, 1].legend(['Average', 'Maximum', 'Minimum'])
  
  [a, b]= np.polyfit(x, tam_q, 1)
  y1 = a * x + b
  axes[1, 0].plot(x, tam_q, 'b')
  axes[1, 0].plot(x, y1, 'r')
  axes[1, 0].grid(True)
  axes[1, 0].set_xlabel('Stresser Hz')
  axes[1, 0].set_ylabel('Queue size')
  axes[1, 0].legend(['Maximum queue size', 'Linear fit'])
  # axes[1, 0].legend(['Maximum queue size'])

  fig.delaxes(axes[1, 1])
  plt.tight_layout()
  plt.show(block=False)

path = '/results'
path = os.getcwd() + path

data = pd.read_csv(path +'/bf_experiments.csv')
# print(data.head()) 
shape = data.shape

# nodes
index = 0
x = data.iloc[index].astype(int).to_numpy()
index = index + 8;
avg_wt = data.iloc[index].astype(float).to_numpy()
max_wt = data.iloc[index + 2].astype(float).to_numpy()
avg_succ = data.iloc[index + 4].astype(float).to_numpy()
max_succ = data.iloc[index + 6].astype(float).to_numpy()
min_succ = data.iloc[index + 7].astype(float).to_numpy()
tam_q = data.iloc[index + 18].astype(float).to_numpy()
plot_results_nodes(x, avg_wt, max_wt, avg_succ, max_succ, min_succ, tam_q)

# time
index = 58
x = data.iloc[index].dropna().astype(int).to_numpy()
index = index + 4;
avg_wt = data.iloc[index].dropna().astype(float).to_numpy()
max_wt = data.iloc[index + 2].dropna().astype(float).to_numpy()
avg_succ = data.iloc[index + 4].dropna().astype(float).to_numpy()
max_succ = data.iloc[index + 6].dropna().astype(float).to_numpy()
min_succ = data.iloc[index + 7].dropna().astype(float).to_numpy()
tam_q = data.iloc[index + 18].dropna().astype(float).to_numpy()
plot_results_time2(x, avg_wt, max_wt, avg_succ, max_succ, min_succ, tam_q)


# frequency
index = 58
x = data.iloc[58].dropna().astype(int).to_numpy()
index = index + 4;
avg_wt = data.iloc[index].dropna().astype(float).to_numpy()
max_wt = data.iloc[index + 2].dropna().astype(float).to_numpy()
avg_succ = data.iloc[index + 4].dropna().astype(float).to_numpy()
max_succ = data.iloc[index + 6].dropna().astype(float).to_numpy()
min_succ = data.iloc[index + 7].dropna().astype(float).to_numpy()
tam_q = data.iloc[index + 18].dropna().astype(float).to_numpy()
plot_results_freq(x, avg_wt, max_wt, avg_succ, max_succ, min_succ, tam_q)


input()
