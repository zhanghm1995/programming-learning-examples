'''
Copyright (c) 2022 by Haiming Zhang. All Rights Reserved.

Author: Haiming Zhang
Date: 2022-07-14 11:52:28
Email: haimingzhang@link.cuhk.edu.cn
Description: The dataset class to read images from folders.
'''

'''
Copyright (c) 2022 by Haiming Zhang. All Rights Reserved.

Author: Haiming Zhang
Date: 2022-07-14 09:18:44
Email: haimingzhang@link.cuhk.edu.cn
Description: The dataset class to load image folder
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
        img = img.resize(self.img_size, Image.BILINEAR)

        img_tensor = self.to_tensor(img)
        
        data_dict = {}
        data_dict['image_tensor'] = img_tensor
        data_dict['image_name'] = img_name
        data_dict['image_origin'] = img

        return data_dict

    def __len__(self):
        return len(self.image_list)

    def __repr__(self):
        return self.__class__.__name__ + '(' + self.folder_path + ')'


def collate_batch_fn(batch):
    data_list = []
    image_name_list = []
    image_origin_list = []
    for data in batch:
        data_list.append(data['image_tensor'])
        image_name_list.append(data['image_name'])
        image_origin_list.append(data['image_origin'])
    
    data_tensor = torch.stack(data_list)

    res_dict = {}
    res_dict['image_tensor'] = data_tensor
    res_dict['image_name'] = image_name_list
    res_dict['image_origin'] = image_origin_list
    return res_dict


if __name__ == "__main__":
    dataset = ImageFolderDataset('/221019051/Datasets/Face/HDTF_preprocessed/RD_Radio34_007/face_image')
    print(len(dataset))

    data = dataset[0]
    print(data['image_name'])

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
