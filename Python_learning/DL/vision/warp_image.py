'''
Copyright (c) 2022 by Haiming Zhang. All Rights Reserved.

Author: Haiming Zhang
Date: 2022-03-12 00:01:20
Email: haimingzhang@link.cuhk.edu.cn
Description: Warp image by using Pytorch
Reference: https://gist.github.com/sftekin/d5ddf65e54e3dfe89a308c624e62280b
'''

"""
https://discuss.pytorch.org/t/solved-torch-grid-sample/51662
I have used this code.
"""

import torch
import torch.nn.functional as F
from torchvision import transforms
from PIL import Image
from PIL import ImageFile

ImageFile.LOAD_TRUNCATED_IMAGES = True

im_path = './data/obama.jpg'
image = Image.open(im_path)
image = image.convert('RGB')

im_transform = transforms.Compose([
    transforms.Resize(224),
    transforms.ToTensor()
])
image = im_transform(image)
# Add batch dimension
image = image.unsqueeze(dim=0)

d = torch.linspace(-1, 1, 224)
meshx, meshy = torch.meshgrid((d, d))

# Just to see the effect
meshx = meshx * 0.01
meshy = meshy * 0.01

grid = torch.stack((meshy, meshx), 2)
grid = grid.unsqueeze(0)
warped = F.grid_sample(image, grid, mode='bilinear')

to_image = transforms.ToPILImage()
# to_image(image.squeeze()).show()
# to_image(warped.squeeze()).show(title='WARPED')

to_image(image.squeeze()).save("origin.jpg")
to_image(warped.squeeze()).save("warped.jpg")

