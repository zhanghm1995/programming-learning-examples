'''
Copyright (c) 2022 by Haiming Zhang. All Rights Reserved.

Author: Haiming Zhang
Date: 2022-12-05 19:48:52
Email: haimingzhang@link.cuhk.edu.cn
Description: Learn the optimizer and learning rate scheduler.
'''

import torch
import torch.nn as nn
from torch.optim.lr_scheduler import ReduceLROnPlateau, StepLR, CosineAnnealingWarmRestarts, \
                                     CosineAnnealingLR, ExponentialLR


model = nn.Conv2d(3, 64, 3)
optimizer = torch.optim.Adam(model.parameters(), lr=0.001)
lr_scheduler = ExponentialLR(optimizer, gamma=0.7)


def get_lr(optimizer):
    """Get the current learning rate from optimizer.
        """
    for param_group in optimizer.param_groups:
        return param_group['lr']


for i in range(10):
    lr_scheduler.step()
    print(get_lr(optimizer))
    print(lr_scheduler.get_lr())
    # print(lr_scheduler.get_last_lr())

