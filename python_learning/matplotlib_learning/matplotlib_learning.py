"""
    Draw scatter points from data.txt
"""
import numpy as np
import matplotlib.pyplot as plt


data = np.loadtxt("data.txt")
print(data)

num = len(data)

x = np.arange(0, num)
plt.figure('data')
plt.plot(x,data,'.')
plt.show()


