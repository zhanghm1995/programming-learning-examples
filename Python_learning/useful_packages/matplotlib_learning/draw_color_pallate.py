'''
Copyright (c) 2024 by Haiming Zhang. All Rights Reserved.

Author: Haiming Zhang
Date: 2024-07-05 10:13:45
Email: haimingzhang@link.cuhk.edu.cn
Description: 
'''
import os
import os.path as osp
import numpy as np
import pickle
import torch

import matplotlib.pyplot as plt
import numpy as np

# 颜色定义
OCC3D_PALETTE = torch.Tensor([
    [0, 0, 0],
    [255, 120, 50],  # barrier              orangey
    [255, 192, 203],  # bicycle              pink
    [255, 255, 0],  # bus                  yellow
    [0, 150, 245],  # car                  blue
    [0, 255, 255],  # construction_vehicle cyan
    [200, 180, 0],  # motorcycle           dark orange
    [255, 0, 0],  # pedestrian           red
    [255, 240, 150],  # traffic_cone         light yellow
    [135, 60, 0],  # trailer              brown
    [160, 32, 240],  # truck                purple
    [255, 0, 255],  # driveable_surface    dark pink
    [139, 137, 137],  # other_flat           dark grey
    [75, 0, 75],  # sidewalk             dard purple
    [150, 240, 80],  # terrain              light green
    [230, 230, 250],  # manmade              white
    [0, 175, 0],  # vegetation           green
    [0, 255, 127],  # ego car              dark cyan
    [255, 99, 71],
    [0, 191, 255],
    [125, 125, 125]
])

# 类别名
categories = ['background', 'barrier', 'bicycle', 'bus', 'car', 'construction_vehicle', 
              'motorcycle', 'pedestrian', 'traffic_cone', 'trailer', 'truck', 
              'driveable_surface', 'other_flat', 'sidewalk', 'terrain', 'manmade', 
              'vegetation', 'ego car']

# 创建画布
fig, ax = plt.subplots()

# 绘制颜色矩形块和文本
for i in range(len(OCC3D_PALETTE)):
    color = OCC3D_PALETTE[i] / 255.0
    rect = plt.Rectangle((0, i), 1, 1, color=color)
    ax.add_patch(rect)
    ax.text(1.1, i+0.5, categories[i], va='center')

# 设置图像样式
ax.set_xlim(0, 2)
ax.set_ylim(0, len(OCC3D_PALETTE))
ax.axis('off')

plt.show()