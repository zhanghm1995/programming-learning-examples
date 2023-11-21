'''
Copyright (c) 2023 by Haiming Zhang. All Rights Reserved.

Author: Haiming Zhang
Date: 2023-11-01 10:49:11
Email: haimingzhang@link.cuhk.edu.cn
Description: 
'''


from mmdet.models.losses import CrossEntropyLoss

import torch
from torch import nn
import torch.nn.functional as F

pred_score = torch.tensor([[13., 3., 2., 5., 1.],
                               [1., 8., 20., 2., 3.]])
target = torch.tensor([3, 1])

class_weight = [0.1, 0.2, 0.3, 0.4, 0.5]
class_weight = torch.tensor(class_weight)

def method1():
    """https://pytorch.org/docs/stable/generated/torch.nn.functional.cross_entropy.html
    """
    loss = F.cross_entropy(pred_score, target, ignore_index=255)
    print(loss)


def method2():
    loss = nn.CrossEntropyLoss(weight=class_weight)
    loss_ce = loss(pred_score, target)
    print(loss_ce)


def method3():
    loss = CrossEntropyLoss(use_sigmoid=False, class_weight=class_weight, reduction='mean')
    loss_ce = loss(pred_score, target)
    print(loss_ce)


# pred_score = torch.tensor([[13., 3., 2., 5., 1.],
#                            [1., 8., 20., 2., 3.],
#                            [1., 14., 3., 5., 3.]])
# print(pred_score)
# pred_score_soft = F.softmax(pred_score, dim=1)
# print(pred_score_soft)
# pred_score_soft_log = pred_score_soft.log()

# target = torch.tensor([3, 1, 255])
# loss = F.cross_entropy(pred_score, target, ignore_index=255)
# print(loss)

method1()
method2()
method3()