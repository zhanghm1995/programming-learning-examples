'''
Copyright (c) 2022 by Haiming Zhang. All Rights Reserved.

Author: Haiming Zhang
Date: 2022-03-15 15:24:40
Email: haimingzhang@link.cuhk.edu.cn
Description: 
'''

import torchvision
import torch
from torchvision import transforms
import torchvision.transforms.functional as TF
from PIL import Image


image_path = "./data/obama.jpg"

image = Image.open(image_path)

TRAIN_IMAGE_SIZE = 256
import numpy as np

image = torch.from_numpy(np.array(image, np.uint8))
image = image.permute(2, 0, 1)

print(type(image), image.shape)

custom_transform = transforms.CenterCrop((TRAIN_IMAGE_SIZE, TRAIN_IMAGE_SIZE))
image = custom_transform(image)

print(image.shape, type(image))