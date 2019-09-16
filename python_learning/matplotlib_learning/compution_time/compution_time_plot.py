"""
    Using python to draw tracking results compared with groundtruth
"""

import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties as FP
import numpy as np
import matplotlib

efp = FP('Times New Roman',size=13)

#### load groundtruth txt file
compute_time = np.loadtxt("computation_time.txt")

detection = compute_time[:,0]
match = compute_time[:,1]
motion = compute_time[:,2]
tracking = compute_time[:,3]
whole = compute_time[:,4]

x_range = np.arange(len(detection))

### ------Begin plot-------
#-------------whole processing time-------
whole_mean = np.mean(whole)
print(whole_mean)
whole_mean = np.ones(len(x_range))*whole_mean

plt.figure('whole_time')
plt.grid(linestyle="-.")
plt.plot(x_range, whole, color = "red", label="每帧总耗时")
plt.plot(x_range, whole_mean, color = "blue", linewidth=3, label="平均耗时")
plt.xlabel("帧数", fontsize=15)
plt.ylabel("耗时[ms]", fontsize=15)
ax = plt.gca()
plt.legend(fontsize=15)
plt.tick_params(labelsize=15)
# plt.savefig("position.svg", format='svg')
plt.show()


#-------------pie chart for different modules time-------
labels = '目标检测(29.83ms)', '数据关联(2.19ms)', '运动更新(0.14ms)'
detection_mean = np.mean(detection)
match_mean = np.mean(match)
motion_mean = np.mean(motion)
print(detection_mean, match_mean, motion_mean)
fracs = [detection_mean, match_mean, motion_mean]
explode = [0, 0, 0.1] # 0.1 凸出这部分，
plt.axes(aspect=1)  # set this , Figure is round, otherwise it is an ellipse
#autopct ，show percet
patches,l_text,p_text = plt.pie(x=fracs, labels=labels,autopct='%3.1f%%',textprops = {'fontsize':15, 'color':'k'},
        shadow=False, labeldistance=1.05, startangle = 90,pctdistance = 0.6, explode=explode
        )
#改变文本的大小
#方法是把每一个text遍历。调用set_size方法设置它的属性
for t in l_text:
    t.set_size(15)
# 设置x，y轴刻度一致，这样饼图才能是圆的
plt.axis('equal')

# for i, p in enumerate(wedges):
#     ang = (p.theta2 - p.theta1)/2. + p.theta1
#     y = np.sin(np.deg2rad(ang))
#     x = np.cos(np.deg2rad(ang))
#     horizontalalignment = {-1: "right", 1: "left"}[int(np.sign(x))]
#     connectionstyle = "angle,angleA=0,angleB={}".format(ang)
#     kw["arrowprops"].update({"connectionstyle": connectionstyle})
#     ax.annotate(labels[i], xy=(x, y), xytext=(1.35*np.sign(x), 1.4*y),
#                  horizontalalignment=horizontalalignment, **kw)
'''
labeldistance，文本的位置离远点有多远，1.1指1.1倍半径的位置
autopct，圆里面的文本格式，%3.1f%%表示小数有三位，整数有一位的浮点数
shadow，饼是否有阴影
startangle，起始角度，0，表示从0开始逆时针转，为第一块。一般选择从90度开始比较好看
pctdistance，百分比的text离圆心的距离
patches, l_texts, p_texts，为了得到饼图的返回值，p_texts饼图内部文本的，l_texts饼图外label的文本
'''
# plt.savefig("computation.svg")
plt.show()

# plt.figure("heading")
# plt.grid(linestyle="-.")
# x_heading = np.arange(len(gt_heading))
# plt.plot(x_heading, gt_heading, color = "red", marker='.', label="GT")
# # plt.plot(tr_pos_x, tr_pos_y, color = "blue", marker='D', fillstyle = "none", label="Track")
# plt.plot(x_heading, tr_heading, color = "blue", linestyle="-.", label="Track")
# plt.title("UKF States")
# plt.xlabel("Frames", fontsize=12, fontproperties=efp)
# plt.ylabel("Yaw [rad]", fontsize=12, fontproperties=efp)
# ax = plt.gca()
# plt.legend(fontsize=15)

# plt.savefig("heading.svg", format='svg')
# # plt.show()

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



# ## Subplots
# x = np.arange(len(gt_pos_x))
# plt.subplot(5, 1, 1)
# plt.plot(x, gt_pos_x, '-*', color = "C1", label="GT")
# plt.plot(x, tr_pos_x, '-', color = "C0", label="Track")
# plt.title('UKF States', fontsize=16, fontweight='bold', fontname="Times New Roman")
# plt.ylabel('X Position [m]', fontproperties=efp)
# plt.legend(fontsize=13, ncol=2)

# plt.subplot(5, 1, 2)
# plt.plot(x, gt_pos_y, '-*', color = "C1")
# plt.plot(x, tr_pos_y, '-', color = "C0")
# plt.ylabel('Y Position [m]', fontproperties=efp)

# plt.subplot(5, 1, 3)
# plt.plot(x, tr_v, '-')
# plt.ylabel('Velocity [m/s]', fontproperties=efp)

# plt.subplot(5, 1, 4)
# plt.plot(x, gt_heading, '-*', color="C1")
# plt.plot(x, tr_heading, '-', color="C0")
# plt.ylabel('Yaw [rad]', fontproperties=efp)

# plt.subplot(5, 1, 5)
# plt.plot(x, tr_yaw_rate, '-')
# plt.ylabel('Yaw Rate [rad/s]', fontproperties=efp)
# plt.xlabel('Frames', fontsize=13, fontproperties=efp)
# plt.savefig("ukf.svg", format='svg')
# plt.show()