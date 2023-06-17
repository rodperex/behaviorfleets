import os
import pandas as pd
import numpy as np
# from scipy.optimize import curve_fit
import matplotlib.pyplot as plt

path = '/results'
path = os.getcwd() + path

data = pd.read_csv(path +'/bf_experiments.csv')
print(data.head()) 
shape = data.shape

# nodes
x = data.iloc[0].astype(int).to_numpy()
avg_wt = data.iloc[8].astype(float).to_numpy()

[a, b]= np.polyfit(x, avg_wt, 1)
y1 = a * x + b

plt.plot(x, avg_wt, 'b')
plt.plot(x, y1, 'r')
plt.show()


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

