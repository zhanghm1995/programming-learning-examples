"""
 Learn how to use subplot to draw multiple plots in one figure
 References: https://matplotlib.org/gallery/subplots_axes_and_figures/subplot.html
"""
import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties as FP
import numpy as np

mpl.rcParams['font.family'] = 'sans-serif'
mpl.rcParams['font.sans-serif'] = 'NSimSun,Times New Roman'
efp = FP('Times New Roman')
x1 = np.linspace(0.0, 5.0)
x2 = np.linspace(0.0, 2.0)

y1 = np.cos(2 * np.pi * x1) * np.exp(-x1)
y2 = np.cos(2 * np.pi * x2)

plt.subplot(2, 1, 1)
plt.plot(x1, y1, 'o-')
plt.title('A tale of 2 subplots',fontweight='bold', fontproperties=efp)
plt.ylabel('Damped oscillation', fontproperties=efp)

plt.subplot(2, 1, 2)
plt.plot(x2, y2, '.-')
plt.xlabel('time [s]')
plt.ylabel('Undamped', fontproperties=efp)

plt.show()