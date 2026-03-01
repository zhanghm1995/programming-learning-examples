'''
Copyright (c) 2024 by Haiming Zhang. All Rights Reserved.

Author: Haiming Zhang
Date: 2024-03-04 11:34:11
Email: haimingzhang@link.cuhk.edu.cn
Description: 
'''


import torch
import torch.nn as nn
import mmcv
import mmdet
from mmdet.models.backbones import ResNet
from mmdet.models.necks import FPN
from mmdet.models.losses import FocalLoss, DiceLoss, CrossEntropyLoss
from mmdet3d.datasets.pipelines.loading import LoadMultiViewImageFromFiles

model = ResNet(50)
neck = FPN(in_channels=[256, 512, 1024, 2048], out_channels=256, num_outs=5)