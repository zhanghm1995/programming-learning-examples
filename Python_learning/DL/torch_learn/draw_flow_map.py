'''
Copyright (c) 2024 by Haiming Zhang. All Rights Reserved.

Author: Haiming Zhang
Date: 2024-11-22 10:33:35
Email: haimingzhang@link.cuhk.edu.cn
Description: 
'''
import os
import os.path as osp
from tqdm import tqdm
import numpy as np
import pickle
import torch
from PIL import Image  
import matplotlib.pyplot as plt
from einops import rearrange
from matplotlib.colors import Normalize 

flow_volume = torch.load("/mnt/data2/zhanghm/Code/Occupancy/Uni3DGS/results/CVPR/voxel_flow_pred_f587.pth")
feat_volume = torch.load("/mnt/data2/zhanghm/Code/Occupancy/Uni3DGS/results/CVPR/volume_feature_f587.pth")

flow_volume = flow_volume.squeeze()
feat_volume = rearrange(feat_volume, 'b x y z c -> b c x y z')
print(flow_volume.shape)
print(feat_volume.shape)


flow_volume = rearrange(flow_volume, 'x y z c -> y x z c')  # change to ego direction, x is the forward direction

for i in range(5):
    flow_map = flow_volume[:, :, i].cpu().numpy()
    flow_map = np.linalg.norm(flow_map, axis=-1)
    # threshold = 0.5  # 设置速度阈值  
    # flow_map[flow_map < threshold] = 0  # 小于阈值的速度置为 0  

    print(flow_map.shape, flow_map.min(), flow_map.max())

    # 2. 归一化速度大小到 [0, 1]
    norm = Normalize(vmin=0, vmax=np.max(flow_map))  # 根据速度范围归一化  
    speed_normalized = norm(flow_map)  

    # 3. 应用 viridis 颜色映射  
    cmap = plt.cm.viridis  # 使用 viridis 颜色映射  
    colored_image = cmap(speed_normalized)  # 将归一化速度映射为 RGBA 图像  

    # 4. 显示结果  
    plt.figure(figsize=(8, 6))  
    plt.imshow(colored_image, origin='upper')  
    plt.title("Flow Map Visualization with Viridis Colormap")  
    plt.axis('off')  # 隐藏坐标轴  
    plt.colorbar(plt.cm.ScalarMappable(norm=norm, cmap=cmap), label="Speed Magnitude")  
    plt.savefig(f'flow_map_{i}.png')  # 保存为图片

# feature_map = feat_volume[0][..., 0]  # 取第一个样本的特征图

# # # 可视化第一个通道的特征图  
# feature_map_channel = feature_map[0].cpu().numpy()  
# feature_map_normalized = (feature_map_channel - feature_map_channel.min()) / (feature_map_channel.max() - feature_map_channel.min())  

# plt.figure(figsize=(6, 6))  
# plt.imshow(feature_map_normalized, cmap='viridis')  
# plt.axis('off')  
# # plt.show()
# plt.savefig('volume_feature_map.png')  # 保存为图片