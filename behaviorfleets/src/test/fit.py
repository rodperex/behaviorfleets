import os
import math
import pandas as pd
import numpy as np
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt

def func(x, a, b):
  return a / np.sqrt(x) + b


path = '/results'
path = os.getcwd() + path

data = pd.read_csv(path +'/bf_experiments.csv')
# print(data.head()) 
shape = data.shape

# nodes
x = data.iloc[0].astype(int).to_numpy()
avg_wt = data.iloc[8].astype(float).to_numpy()
max_wt = data.iloc[10].astype(float).to_numpy()
avg_succ = data.iloc[12].astype(float).to_numpy()
max_succ = data.iloc[14].astype(float).to_numpy()
min_succ = data.iloc[15].astype(float).to_numpy()
tam_q = data.iloc[21].astype(float).to_numpy()

# time

# frequency

[a, b]= np.polyfit(x, avg_wt, 1)
y1 = a * x + b
plt.plot(x, avg_wt, 'b')
plt.plot(x, max_wt, 'g')
plt.plot(x, y1, 'r')
plt.grid(True)
plt.xlabel('Number of nodes')
plt.ylabel('Waiting time')
plt.legend(['Average', 'Maximum', 'Linear fit'])
plt.show(block=False)


popt, pcov = curve_fit(func, x, avg_succ)
[a, b] = popt
y1 =  a / np.sqrt(x) + b
plt.figure()
plt.plot(x, avg_succ, 'b')
plt.plot(x, y1, 'r') 
plt.plot(x, max_succ, 'g')
plt.plot(x, min_succ, 'y')
plt.grid(True)
plt.xlabel('Number of nodes')
plt.ylabel('Success rate')
plt.legend(['Average', 'Curve fit', 'Maximum', 'Minimum'])
plt.show(block=False)

[a, b]= np.polyfit(x, tam_q, 1)
y1 = a * x + b
plt.figure()
plt.plot(x, tam_q, 'b')
plt.plot(x, y1, 'r')
plt.grid(True)
plt.xlabel('Number of nodes')
plt.ylabel('Queue size')
plt.legend(['Maximum queue size', 'Linear fit'])
plt.show(block=False)

input()



# time = [60, 120, 240, 480, 960, 1920]
# tam_q = [54, 92, 196,	385, 773,	1291]

# time = [x/60 for x in time]

# x = np.array(time)
# y = np.array(tam_q)

# [a, b], res1 = curve_fit(lambda x1,a,b: a*np.exp(b*x1),  x,  y)
# y1 = a * np.exp(b * x)

# [a, b]= np.polyfit(x, y, 1  )
# y1 = a * x + b
 
# plt.plot(x, y, 'b')
# plt.plot(x, y1, 'r')
# plt.show()


