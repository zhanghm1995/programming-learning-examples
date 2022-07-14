'''
Copyright (c) 2022 by Haiming Zhang. All Rights Reserved.

Author: Haiming Zhang
Date: 2022-07-08 20:54:36
Email: haimingzhang@link.cuhk.edu.cn
Description: Resize all image in one time.
'''

import os
import os.path as osp
import cv2


def resize_images(root_dir, resize_size):
    """Resize all images in one time.
    
    Args:
        root_dir (str): the root path of the images.
        resize_size (int): the size of the resized image.
    """
    all_files = os.listdir(root_dir)
    for file in all_files:
        input_file = osp.join(root_dir, file)
        if osp.isdir(input_file) or file.endswith(".txt"):
            continue
        img = cv2.imread(input_file)
        img = cv2.resize(img, (resize_size, resize_size))
        cv2.imwrite(input_file, img)


if __name__ == "__main__":
    root_dir = "/home/zhanghm/Temp/cv-fighter/face_fighter/data"
    resize_size = 256
    resize_images(root_dir, resize_size)