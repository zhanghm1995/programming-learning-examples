'''
Copyright (c) 2022 by Haiming Zhang. All Rights Reserved.

Author: Haiming Zhang
Date: 2022-02-27 20:02:48
Email: haimingzhang@link.cuhk.edu.cn
Description: 
'''

import os.path as osp
import numpy as np

# npz_file_path = "/home/haimingzhang/Research/Face/FaceFusionFormer/work_dir/train_3dmm_facial_HDTF/lightning_logs/version_2/vis/000.npz"
# netparams = np.load(open(npz_file_path, 'rb'))

# print(netparams.files)  ## Get the dictionary keys

# content = netparams['face']

# content = content[1:, :] - content[:-1, :]
# print(content.shape)

# exit(0)

def stat_deep3d_face_params(data_array):
    min_value = np.min(data_array, axis=0)
    max_value = np.max(data_array, axis=0)

    ## motion information
    array_diff = data_array[1:, :] - data_array[:-1, :]
    min_value = array_diff.min()

    choose_base = data_array[3:4, :]
    diff = data_array - choose_base
    min_value = diff.min()



def read_all_deep3dface_params(data_root, split_file):
    all_videos_dir = open(split_file).read().splitlines()
    
    all_deep3dface_params = []

    for dir in all_videos_dir:
        params_file = osp.join(data_root, dir, "deep3dface.npz")
        netparams = np.load(open(params_file, 'rb'))
        content = netparams['face']
        all_deep3dface_params.append(content)
    all_deep3dface_params_arr = np.concatenate(all_deep3dface_params, axis=0) # (N, 64)
    return all_deep3dface_params_arr

# all_deep3dface_params_arr = read_all_deep3dface_params(
#     "/home/haimingzhang/Research/Face/FaceFormer/FaceFormer/data/id00002", 
#     "/home/haimingzhang/Research/Face/FaceFormer/FaceFormer/data/id00002/all.txt")

npz_file = "/home/haimingzhang/Research/Face/FaceFusionFormer/data/HDTF_preprocessed/RD_Radio7_000/deep3dface.npz"
face_3d_params = np.load(open(npz_file, 'rb'))['face']

stat_deep3d_face_params(face_3d_params)


