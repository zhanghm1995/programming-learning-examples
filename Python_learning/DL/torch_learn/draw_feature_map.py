'''
Copyright (c) 2024 by Haiming Zhang. All Rights Reserved.

Author: Haiming Zhang
Date: 2024-11-21 19:21:41
Email: haimingzhang@link.cuhk.edu.cn
Description: 
'''
import os
import os.path as osp
from tqdm import tqdm
import numpy as np
import pickle
import torch

import torch  
import torch.nn as nn  
import torchvision.models as models  
import torchvision.transforms as transforms  
from PIL import Image  
import matplotlib.pyplot as plt  

# 加载预训练的模型 (以 ResNet 为例)  
model = models.resnet18(pretrained=True)  
model.eval()  

# 定义一个钩子函数来提取中间层的特征图  
features = []  
def hook_fn(module, input, output):  
    features.append(output)  

# 注册钩子到某一层 (例如，第一层卷积层)  
layer = model.conv1  
layer.register_forward_hook(hook_fn)  

# 加载并预处理输入图像  
transform = transforms.Compose([  
    transforms.Resize((224, 224)),  
    transforms.ToTensor(),  
    transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),  
])  
image = Image.open("./data/obama.jpg").convert("RGB")  # 替换为你的图像路径  
input_tensor = transform(image).unsqueeze(0) 
# 前向传播以提取特征   

with torch.no_grad():  
    model(input_tensor)  

# 提取的特征图 (形状: [C, H, W])  
feature_map = features[0][0]  # 取第一个样本的特征图  

print(feature_map.shape)

# 可视化第一个通道的特征图  
feature_map_channel = feature_map[0].cpu().numpy()  
feature_map_normalized = (feature_map_channel - feature_map_channel.min()) / (feature_map_channel.max() - feature_map_channel.min())  

plt.figure(figsize=(6, 6))  
plt.imshow(feature_map_normalized, cmap='viridis')  
plt.axis('off')  
# plt.show()
plt.savefig('feature_map.png')  # 保存为图片