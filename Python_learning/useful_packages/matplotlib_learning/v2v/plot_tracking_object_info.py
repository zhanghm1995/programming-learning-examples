"""
    Using python to draw tracking results compared with groundtruth
"""

import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties as FP
import numpy as np
"""
efp = FP('Times New Roman',size=13)

#### load groundtruth txt file
f = open("/media/zhanghm/zhanghm_ssd/Experiments/data_sync/1020/v2v.txt", "r")
velocity = []
for line in f:
    line = line.strip()
    fields = line.split(" ")
    velocity.append(float(fields[5]))

plt.figure("velocity")
plt.grid(linestyle="-.")
x_heading = np.arange(len(velocity))
plt.plot(x_heading, velocity, color = "red", label="GT")
plt.title("UKF States")
plt.xlabel("Frames", fontsize=12, fontproperties=efp)
plt.ylabel("Yaw [rad]", fontsize=12, fontproperties=efp)
ax = plt.gca()

plt.show()
"""



efp = FP('Times New Roman',size=13)

#### load groundtruth txt file
gt = np.loadtxt("1008_groundtruth.txt")
print(type(gt), gt.shape)

start = 121
end = 390

timestamp = gt[start:end,0]
gt_pos_x = gt[start:end,1]
gt_pos_y = gt[start:end,2]
gt_heading = gt[start:end,4]
gt_velocity = (gt[start:end,5])/3.6


### load track results
track = np.loadtxt("1008_track_results.txt")
tr_pos_x = track[start:end,1]
tr_pos_y = track[start:end,2]
tr_v = track[start:end,3]
tr_v[0]=0.0
tr_heading = track[start:end,4]
tr_yaw_rate = track[start:end,5]

### ------Begin plot-------
plt.figure('position')
plt.grid(linestyle="-.")
plt.plot(gt_pos_x, gt_pos_y, color = "red", marker='*', label="GT")
# plt.plot(tr_pos_x, tr_pos_y, color = "blue", marker='D', fillstyle = "none", label="Track")
plt.plot(tr_pos_x, tr_pos_y, color = "blue", linestyle="-.", label="Track")
plt.title("跟踪目标位置变化")
plt.xlabel("X Position [m]", fontsize=12, fontproperties=efp)
plt.ylabel("Y Position [m]", fontsize=12, fontproperties=efp)
ax = plt.gca()
plt.legend(fontsize=15)

plt.savefig("position.svg", format='svg')
# plt.show()


plt.figure("heading")
plt.grid(linestyle="-.")
x_heading = np.arange(len(gt_heading))
plt.plot(x_heading, gt_heading, color = "red", marker='.', label="GT")
# plt.plot(tr_pos_x, tr_pos_y, color = "blue", marker='D', fillstyle = "none", label="Track")
plt.plot(x_heading, tr_heading, color = "blue", linestyle="-.", label="Track")
plt.title("UKF States")
plt.xlabel("Frames", fontsize=12, fontproperties=efp)
plt.ylabel("Yaw [rad]", fontsize=12, fontproperties=efp)
ax = plt.gca()
plt.legend(fontsize=15)

plt.savefig("heading.svg", format='svg')
# plt.show()

# ## Calculate velocity by hand
# delta_t = []
# timestamp1 = timestamp[1::]
# for idx, t in enumerate(timestamp1):
#     d = t - timestamp[idx]
#     delta_t.append(d)

# delta_gt_pos_x = []
# gt_pos_x1 = gt_pos_x[1::]
# for idx, x in enumerate(gt_pos_x1):
#     d = x - gt_pos_x[idx]
#     delta_gt_pos_x.append(d)



## Subplots
x = np.arange(len(gt_pos_x))
plt.subplot(5, 1, 1)
plt.plot(x, gt_pos_x, '-*', color = "C1", label="GT")
plt.plot(x, tr_pos_x, '-', color = "C0", label="Track")
plt.title('UKF States', fontsize=16, fontweight='bold', fontname="Times New Roman")
plt.ylabel('X Position [m]', fontproperties=efp)
plt.legend(fontsize=13, ncol=2)

plt.subplot(5, 1, 2)
plt.plot(x, gt_pos_y, '-*', color = "C1")
plt.plot(x, tr_pos_y, '-', color = "C0")
plt.ylabel('Y Position [m]', fontproperties=efp)

plt.subplot(5, 1, 3)
plt.plot(x, gt_velocity, '-*', color = "C1")
plt.plot(x, tr_v, '-',color = "C0")
plt.ylabel('Velocity [m/s]', fontproperties=efp)

plt.subplot(5, 1, 4)
plt.plot(x, gt_heading, '-*', color="C1")
plt.plot(x, tr_heading, '-', color="C0")
plt.ylabel('Yaw [rad]', fontproperties=efp)

plt.subplot(5, 1, 5)
plt.plot(x, tr_yaw_rate, '-')
plt.ylabel('Yaw Rate [rad/s]', fontproperties=efp)
plt.xlabel('Frames', fontsize=13, fontproperties=efp)
plt.savefig("ukf.svg", format='svg')
plt.show()