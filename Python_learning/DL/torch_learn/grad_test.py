'''
Copyright (c) 2024 by Haiming Zhang. All Rights Reserved.

Author: Haiming Zhang
Date: 2024-10-25 09:51:00
Email: haimingzhang@link.cuhk.edu.cn
Description: 
'''
import os
import os.path as osp
import numpy as np
import pickle
import torch
import torch.nn as nn


bn = nn.BatchNorm2d(3)

# x = torch.randn(2, 2)
# x.requires_grad = True

# lin0 = nn.Linear(2, 2)
# lin1 = nn.Linear(2, 2)
# lin2 = nn.Linear(2, 2)
# x1 = lin0(x)
# with torch.no_grad():    
#     x2 = lin1(x1)
# x3 = lin2(x2)
# x3.sum().backward()

# print(lin0.weight.requires_grad, lin1.weight.requires_grad, lin2.weight.requires_grad)
# print(lin0.weight.grad, lin1.weight.grad, lin2.weight.grad)

print('-----------')
x = torch.randn(2, 2)
x.requires_grad = True

lin0 = nn.Linear(2, 2)
lin1 = nn.Linear(2, 2)
lin2 = nn.Linear(2, 2)
print(lin0.training, lin1.training, lin2.training)

x1 = lin0(x)
x2 = lin1(x1)
x3 = lin2(x2)
# with torch.no_grad():   
#     x3 = lin2(x2)
x3.sum().backward()


print(lin0.weight.requires_grad, lin1.weight.requires_grad, lin2.weight.requires_grad)
print(lin0.weight.grad, lin1.weight.grad, lin2.weight.grad)