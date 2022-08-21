'''
Copyright (c) 2022 by Haiming Zhang. All Rights Reserved.

Author: Haiming Zhang
Date: 2022-08-10 15:56:50
Email: haimingzhang@link.cuhk.edu.cn
Description: 
'''

import os
import os.path as osp
from glob import glob
from tqdm import tqdm
import time
import numpy as np

import cv2
from PIL import Image

import torch
import torch.nn as nn
import torchvision
import torchvision.transforms as transforms
from torch.utils.data import Dataset

from kornia import morphology as morph

device = torch.device("cuda")

kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (41, 41))
kernel = torch.from_numpy(kernel.astype(np.int64)).to(device)


def apply_morpho_operation(save_dir="./temp_results"):
    os.makedirs(save_dir, exist_ok=True)

    root_dir = "/home/zhanghm/Research/Face/PIRender/temp_results"
    file_list = sorted(glob(osp.join(root_dir, "*.png")))
    
    for file_path in tqdm(file_list):
        mask_img = Image.open(file_path).convert("RGB")
        mask_img = transforms.ToTensor()(mask_img)
        mask_img = mask_img.to(device)
        
        morpho_image = morph.closing(mask_img[None], kernel)
        
        morpho_image *= 255.0
        mask = morpho_image[0].permute(1, 2, 0).cpu().numpy()
        mask = mask.astype(np.uint8)
        
        file_name = osp.basename(file_path).replace(".png", ".png")
        cv2.imwrite(osp.join(save_dir, file_name), mask)


apply_morpho_operation()