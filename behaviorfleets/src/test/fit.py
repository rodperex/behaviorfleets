import os
import math
import pandas as pd
import numpy as np
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt

def func(x, a, b):
  return a / np.sqrt(x) + b


def plot_results_nodes(x, avg_wt, max_wt, avg_succ, max_succ, min_succ, tam_q):
  fig, axes = plt.subplots(nrows=2, ncols=2)

  [a, b] = np.polyfit(x, avg_wt, 1)
  y1 = a * x + b
  # plt.figure()
  # plt.plot(x, avg_wt, 'b')
  # plt.plot(x, max_wt, 'g')
  # plt.plot(x, y1, 'r')
  # plt.grid(True)
  # plt.xlabel('Number of nodes')
  # plt.ylabel('Waiting time')
  # plt.legend(['Average', 'Maximum', 'Linear fit'])
  # plt.show(block=False)
  axes[0, 0].plot(x, avg_wt, 'b')
  axes[0, 0].plot(x, max_wt, 'g')
  axes[0, 0].plot(x, y1, 'r')
  axes[0, 0].grid(True)
  axes[0, 0].set_xlabel('Number of nodes')
  axes[0, 0].set_ylabel('Waiting time')
  axes[0, 0].legend(['Average', 'Maximum', 'Linear fit'])


  [a, b], pcov = curve_fit(func, x, avg_succ)
  y1 =  a / np.sqrt(x) + b
  # plt.figure()
  # plt.plot(x, avg_succ, 'b')
  # plt.plot(x, y1, 'r') 
  # plt.plot(x, max_succ, 'g')
  # plt.plot(x, min_succ, 'y')
  # plt.grid(True)
  # plt.xlabel('Number of nodes')
  # plt.ylabel('Success rate')
  # plt.legend(['Average', 'Curve fit', 'Maximum', 'Minimum'])
  # plt.show(block=False)
  axes[0, 1].plot(x, avg_succ, 'b')
  axes[0, 1].plot(x, y1, 'r')
  axes[0, 1].plot(x, max_succ, 'g')
  axes[0, 1].plot(x, min_succ, 'y')
  axes[0, 1].grid(True)
  axes[0, 1].set_xlabel('Number of nodes')
  axes[0, 1].set_ylabel('Success rate')
  axes[0, 1].legend(['Average', 'Curve fit', 'Maximum', 'Minimum'])


  [a, b]= np.polyfit(x, tam_q, 1)
  y1 = a * x + b
  # plt.figure()
  # plt.plot(x, tam_q, 'b')
  # plt.plot(x, y1, 'r')
  # plt.grid(True)
  # plt.xlabel('Number of nodes')
  # plt.ylabel('Queue size')
  # plt.legend(['Maximum queue size', 'Linear fit'])
  # plt.show(block=False)
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
  nan_indices = np.isnan(avg_wt)
  avg_wt = avg_wt[~nan_indices]
  max_wt = max_wt[~nan_indices]
  avg_succ = avg_succ[~nan_indices]
  max_succ = max_succ[~nan_indices]
  min_succ = min_succ[~nan_indices]
  tam_q = tam_q[~nan_indices]
  x = x[~nan_indices]

  fig, axes = plt.subplots(nrows=2, ncols=2)

  [a, b] = np.polyfit(x, avg_wt, 1)
  y1 = a * x + b
  axes[0, 0].plot(x, avg_wt, 'b')
  axes[0, 0].plot(x, max_wt, 'g')
  axes[0, 0].plot(x, y1, 'r')
  axes[0, 0].grid(True)
  axes[0, 0].set_xlabel('Operation time')
  axes[0, 0].set_ylabel('Waiting time')
  axes[0, 0].legend(['Average', 'Maximum', 'Linear fit'])


  [a, b] = np.polyfit(x, avg_wt, 1)
  y1 = a * x + b
  axes[0, 1].plot(x, avg_succ, 'b')
  axes[0, 1].plot(x, y1, 'r')
  axes[0, 1].plot(x, max_succ, 'g')
  axes[0, 1].plot(x, min_succ, 'y')
  axes[0, 1].grid(True)
  axes[0, 1].set_xlabel('Operation time')
  axes[0, 1].set_ylabel('Success rate')
  axes[0, 1].legend(['Average', 'Linear fit', 'Maximum', 'Minimum'])
  
  [a, b]= np.polyfit(x, tam_q, 1)
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

  [a, b] = np.polyfit(x, avg_wt, 1)
  y1 = a * x + b
  axes[0, 0].plot(x, avg_wt, 'b')
  axes[0, 0].plot(x, max_wt, 'g')
  axes[0, 0].plot(x, y1, 'r')
  axes[0, 0].grid(True)
  axes[0, 0].set_xlabel('Stresser Hz')
  axes[0, 0].set_ylabel('Waiting time')
  axes[0, 0].legend(['Average', 'Maximum', 'Linear fit'])
  

  [a, b] = np.polyfit(x, avg_wt, 1)
  y1 = a * x + b
  axes[0, 1].plot(x, avg_succ, 'b')
  axes[0, 1].plot(x, y1, 'r')
  axes[0, 1].plot(x, max_succ, 'g')
  axes[0, 1].plot(x, min_succ, 'y')
  axes[0, 1].grid(True)
  axes[0, 1].set_xlabel('Stresser Hz')
  axes[0, 1].set_ylabel('Success rate')
  axes[0, 1].legend(['Average', 'Linear fit', 'Maximum', 'Minimum'])
  
  [a, b]= np.polyfit(x, tam_q, 1)
  y1 = a * x + b
  axes[1, 0].plot(x, tam_q, 'b')
  axes[1, 0].plot(x, y1, 'r')
  axes[1, 0].grid(True)
  axes[1, 0].set_xlabel('Stresser Hz')
  axes[1, 0].set_ylabel('Queue size')
  axes[1, 0].legend(['Maximum queue size', 'Linear fit'])

  fig.delaxes(axes[1, 1])
  plt.tight_layout()
  plt.show(block=False)

path = '/results'
path = os.getcwd() + path

data = pd.read_csv(path +'/bf_experiments.csv')
# print(data.head()) 
shape = data.shape

# nodes
index = 8
x = data.iloc[0].astype(int).to_numpy()
avg_wt = data.iloc[index].astype(float).to_numpy()
max_wt = data.iloc[index + 2].astype(float).to_numpy()
avg_succ = data.iloc[index + 4].astype(float).to_numpy()
max_succ = data.iloc[index + 6].astype(float).to_numpy()
min_succ = data.iloc[index + 7].astype(float).to_numpy()
tam_q = data.iloc[index + 13].astype(float).to_numpy()
plot_results_nodes(x, avg_wt, max_wt, avg_succ, max_succ, min_succ, tam_q)

# time
index = 30
x = data.iloc[24].astype(int).to_numpy()
avg_wt = data.iloc[index].astype(float).to_numpy()
max_wt = data.iloc[index + 2].astype(float).to_numpy()
avg_succ = data.iloc[index + 4].astype(float).to_numpy()
max_succ = data.iloc[index + 6].astype(float).to_numpy()
min_succ = data.iloc[index + 7].astype(float).to_numpy()
tam_q = data.iloc[index + 13].astype(float).to_numpy()
plot_results_time(x, avg_wt, max_wt, avg_succ, max_succ, min_succ, tam_q)


# frequency
index = 52
x = data.iloc[48].astype(int).to_numpy()
avg_wt = data.iloc[index].astype(float).to_numpy()
max_wt = data.iloc[index + 2].astype(float).to_numpy()
avg_succ = data.iloc[index + 4].astype(float).to_numpy()
max_succ = data.iloc[index + 6].astype(float).to_numpy()
min_succ = data.iloc[index + 7].astype(float).to_numpy()
tam_q = data.iloc[index + 13].astype(float).to_numpy()
plot_results_freq(x, avg_wt, max_wt, avg_succ, max_succ, min_succ, tam_q)


input()






