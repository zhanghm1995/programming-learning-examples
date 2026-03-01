'''
Copyright (c) 2024 by Haiming Zhang. All Rights Reserved.

Author: Haiming Zhang
Date: 2024-11-05 00:49:43
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
import torch.optim as optim  
from torch.utils.data import DataLoader, TensorDataset  
import os  

# 初始化分布式环境  
def init_distributed_mode():  
    os.environ['MASTER_ADDR'] = 'localhost'  
    os.environ['MASTER_PORT'] = '12345'  
    torch.distributed.init_process_group(backend='nccl')  
    local_rank = torch.distributed.get_rank()  
    torch.cuda.set_device(local_rank)  
    return local_rank  

# 定义一个简单的神经网络  
class SimpleNet(nn.Module):  
    def __init__(self):  
        super(SimpleNet, self).__init__()  
        self.fc1 = nn.Linear(10, 5)  # 这一层会参与训练  
        self.fc2 = nn.Linear(5, 2)    # 这一层会参与训练  
        self.fc3 = nn.Linear(5, 2)    # 这一层不会参与训练 (未使用)  

    def forward(self, x):  
        x = self.fc1(x)  
        x = torch.relu(x)  
        x = self.fc2(x)  
        return x  # 不经过fc3，导致fc3的权重未被使用  

def main():  
    local_rank = init_distributed_mode()  
    
    # 创建数据集  
    train_data = torch.randn(100, 10)  
    train_labels = torch.randint(0, 2, (100, 2)).float()  
    dataset = TensorDataset(train_data, train_labels)  
    train_loader = DataLoader(dataset, batch_size=16, shuffle=True)  

    # 实例化模型并转移到GPU  
    model = SimpleNet().cuda(local_rank)  
    
    # 包装模型以使用 DistributedDataParallel  
    model = nn.parallel.DistributedDataParallel(model, find_unused_parameters=True, device_ids=[local_rank])  

    # 定义损失函数和优化器  
    criterion = nn.BCELoss()  
    optimizer = optim.Adam(model.parameters(), lr=0.01)  

    # 训练循环  
    num_epochs = 10  
    for epoch in range(num_epochs):  
        for inputs, labels in train_loader:  
            inputs, labels = inputs.cuda(local_rank), labels.cuda(local_rank)  

            optimizer.zero_grad()  # 清空梯度  

            # 前向传播  
            outputs = model(inputs)  

            # 计算损失  
            loss = criterion(outputs, labels)  
            
            # 反向传播  
            loss.backward()  
            
            # 更新参数  
            optimizer.step()   

        print(f'Epoch [{epoch + 1}/{num_epochs}], Loss: {loss.item():.4f} - Rank: {local_rank}')  

    # 检查未使用的参数  
    for name, param in model.named_parameters():  
        if param.grad is None:  
            print(f'Parameter {name} has not been used in the training.')  

if __name__ == '__main__':  
    main()