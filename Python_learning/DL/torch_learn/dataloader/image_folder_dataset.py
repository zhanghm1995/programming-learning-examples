'''
Copyright (c) 2022 by Haiming Zhang. All Rights Reserved.

Author: Haiming Zhang
Date: 2022-07-14 11:52:28
Email: haimingzhang@link.cuhk.edu.cn
Description: The dataset class to read images from folders.
https://www.daimajiaoliu.com/daima/4ede05ecd1003fc
'''

import os
import os.path as osp
import numpy as np
import cv2
import torch
from torch.utils.data import Dataset, DataLoader
import torchvision.transforms as transforms
import PIL.Image as Image


IMG_EXTENSIONS = [
    '.jpg', '.JPG', '.jpeg', '.JPEG',
    '.png', '.PNG', '.ppm', '.PPM', '.bmp', '.BMP', '.tiff'
]


def is_image_file(filename):
    """Check a file name is an image file or not.

    Args:
        filename (str|Path): The file name with extension.

    Returns:
        Bool: True if the file is an image file.
    """
    return any(filename.endswith(extension) for extension in IMG_EXTENSIONS)


class ImageFolderDataset(Dataset):
    def __init__(self, folder_path, img_size=(512, 512)):
        self.folder_path = folder_path
        self.img_size = img_size

        self.image_list = sorted([img_file for img_file in os.listdir(folder_path) if is_image_file(img_file)])

        self.to_tensor = transforms.Compose([
            transforms.ToTensor(),
            transforms.Normalize((0.485, 0.456, 0.406), (0.229, 0.224, 0.225)),
        ])

    def __getitem__(self, index):
        img_name = self.image_list[index]

        img_path = osp.join(self.folder_path, img_name)
        
        img = Image.open(img_path).convert('RGB')
        img = img.resize(self.img_size, Image.Resampling.BILINEAR)

        img_tensor = self.to_tensor(img)
        
        data_dict = {}
        data_dict['image_tensor'] = img_tensor
        data_dict['image_name'] = img_name
        data_dict['image_origin'] = None

        return data_dict

    def __len__(self):
        return len(self.image_list)

    def __repr__(self):
        return self.__class__.__name__ + '(' + self.folder_path + ')'


def collate_batch_fn(batch):
    elem = batch[0]

    res_dict = {}
    for key in elem:
        value_list = [d[key] for d in batch]

        value = value_list[0]
        value_type = type(value)
        
        if value is None:
            res_dict[key] = None
        elif isinstance(value, torch.Tensor):
            res_dict[key] = torch.stack(value_list, 0)
        elif isinstance(value, str):
            res_dict[key] = value_list
        else:
            print("Error", key, value_type)

    return res_dict


if __name__ == "__main__":
    image_folder = "/data/data0/zhanghm/Datasets/HDTF_face3dmmformer/train/WDA_BarackObama_000/face_image"
    
    dataset = ImageFolderDataset(image_folder)
    print(len(dataset))

    ## Create dataloader
    data_loader = DataLoader(
        dataset,
        batch_size=10,
        shuffle=False,
        num_workers=8,
        pin_memory=True,
        collate_fn=lambda x: collate_batch_fn(x)
    )
    print(len(data_loader))

    for i, data in enumerate(data_loader):
        print(i, type(data), data['image_tensor'].shape)
        print(data['image_name'])
        print(data['image_origin'])
        break
