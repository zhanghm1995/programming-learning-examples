import numpy as np
import cmath
import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties as FP

gt = np.loadtxt("groundtruth.txt")
print(type(gt), gt.shape)

timestamp = gt[:,0]
gt_pos_x = gt[:,1]
gt_pos_y = gt[:,2]
gt_pos_z = gt[:,3]
gt_heading = gt[:,4]

## Calculate velocity by hand
delta_t = []
timestamp1 = timestamp[1::]
for idx, t in enumerate(timestamp1):
    d = t - timestamp[idx]
    delta_t.append(d)

delta_gt_pos_x = []
gt_pos_x1 = gt_pos_x[1::]
for idx, x in enumerate(gt_pos_x1):
    d = x - gt_pos_x[idx]
    delta_gt_pos_x.append(d)

delta_gt_pos_y = []
gt_pos_y1 = gt_pos_y[1::]
for idx, y in enumerate(gt_pos_y1):
    d = y - gt_pos_y[idx]
    delta_gt_pos_y.append(d)

vx = [x/t for x,t in zip(delta_gt_pos_x,delta_t)]
vy = [y/t for y,t in zip(delta_gt_pos_y,delta_t)]
v = [cmath.sqrt(a**2 + b**2) for a,b in zip(vx, vy)]
v.insert(0,0.0)


### load track results
track = np.loadtxt("track_results.txt")
tr_pos_x = track[:,1]
tr_pos_y = track[:,2]
tr_v = track[:,3]
tr_heading = track[:,4]
tr_yaw_rate = track[:,5]


frame = np.arange(len(v))

mpl.rcParams['font.family'] = 'sans-serif'
mpl.rcParams['font.sans-serif'] = 'SimHei,Times New Roman'
cfp = FP('SimHei', size=15)

plt.figure("v")
plt.grid(linestyle="-.", color='gray')
plt.plot(frame, tr_v*3.6, '-', color='blue', label="UKF输出")
plt.plot(frame, np.array(v)*3.6, color = 'red', label="直接计算")
plt.xlabel("Frames", fontsize=15)
plt.ylabel("Velocity [km/h]", fontsize=15)
plt.legend(prop=cfp, ncol=2)
# plt.savefig("v.svg", format="svg")
plt.show()