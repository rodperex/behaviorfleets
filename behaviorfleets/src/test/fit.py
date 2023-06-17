import numpy as np
# from scipy.optimize import curve_fit
import matplotlib.pyplot as plt

time = [60, 120, 240, 480, 960, 1920]
tam_q = [54, 92, 196,	385, 773,	1291]

# time = [x/60 for x in time]

x = np.array(time)
y = np.array(tam_q)

# [a, b], res1 = curve_fit(lambda x1,a,b: a*np.exp(b*x1),  x,  y)
# y1 = a * np.exp(b * x)

[a, b]= np.polyfit(x, y, 1  )
y1 = a * x + b
 
plt.plot(x, y, 'b')
plt.plot(x, y1, 'r')
plt.show()

