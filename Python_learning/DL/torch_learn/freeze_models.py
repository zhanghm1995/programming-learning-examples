'''
Copyright (c) 2023 by Haiming Zhang. All Rights Reserved.

Author: Haiming Zhang
Date: 2023-06-18 21:05:36
Email: haimingzhang@link.cuhk.edu.cn
Description: Learn the tricks when freezing the model.
Reference: https://blog.csdn.net/grllery/article/details/110351465
'''

# -*- coding:utf-8 -*-
import torch
import torch.nn as nn
import torch.nn.functional as F

class Net(nn.Module):

    def __init__(self):
        super(Net, self).__init__()
        self.conv1 = nn.Conv2d(1, 6, 3)
        self.bn1 = nn.BatchNorm2d(6)
        self.conv2 = nn.Conv2d(6, 16, 3)
        self.bn2 = nn.BatchNorm2d(16)
        # an affine operation: y = Wx + b
        self.fc1 = nn.Linear(16 * 6 * 6, 120)  # 6*6 from image dimension
        self.fc2 = nn.Linear(120, 84)
        self.fc3 = nn.Linear(84, 5)

    def forward(self, x):
        # Max pooling over a (2, 2) window
        x = F.max_pool2d(F.relu(self.bn1(self.conv1(x))), (2, 2))
        # If the size is a square you can only specify a single number
        x = F.max_pool2d(F.relu(self.bn2(self.conv2(x))), 2)
        x = x.view(-1, self.num_flat_features(x))
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = self.fc3(x)
        return x

    def num_flat_features(self, x):
        size = x.size()[1:]  # all dimensions except the batch dimension
        num_features = 1
        for s in size:
            num_features *= s
        return num_features

def print_parameter_grad_info(net):
    print('-------parameters requires grad info--------')
    for name, p in net.named_parameters():
        print(f'{name}:\t{p.requires_grad}')

def print_net_state_dict(net):
    for key, v in net.state_dict().items():
        print(f'{key}')


if __name__ == "__main__":
    net = Net()

    print_parameter_grad_info(net)
    net.eval()
    net.requires_grad_(False)
    print_parameter_grad_info(net)

    torch.random.manual_seed(5)
    test_data = torch.rand(1, 1, 32, 32)
    train_data = torch.rand(5, 1, 32, 32)

    # print(test_data)
    # print(train_data[0, ...])
    for epoch in range(2):
        # training phase, 假设每个epoch只迭代一次
        net.train()
        pre = net(train_data)
        # 计算损失和参数更新等
        # ....

        # test phase
        net.eval()
        x = net(test_data)
        print(f'epoch:{epoch}', x)

