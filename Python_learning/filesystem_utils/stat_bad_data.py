'''
Copyright (c) 2022 by Haiming Zhang. All Rights Reserved.

Author: Haiming Zhang
Date: 2022-03-16 16:01:48
Email: haimingzhang@link.cuhk.edu.cn
Description: 
'''

import os
import os.path as osp
from glob import glob
from tqdm import tqdm


def get_folder_list(root_dir):
    folder_list = [entry.path for entry in os.scandir(root_dir) if entry.is_dir()]
    folder_list = sorted(folder_list)
    return folder_list


data_root = "/home/haimingzhang/Research/Programming/cv-fighter/HDTF_Dataset/HDTF_preprocessed"

folder_list = get_folder_list(data_root)

print(len(folder_list), folder_list[:3])

txt = open("bad_data3.txt", 'w')

for folder in tqdm(folder_list):
    num_face_image = len(os.listdir(osp.join(folder, "face_image")))
    try:
        num_mouth_mask = len(os.listdir(osp.join(folder, "mouth_mask")))
        if num_face_image / 2 != num_mouth_mask:
            txt.write(osp.basename(folder))
            txt.write("\n")
    except:
        txt.write(osp.basename(folder))
        txt.write("\n")
    
txt.close()
