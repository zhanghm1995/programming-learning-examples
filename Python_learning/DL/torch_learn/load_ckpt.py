'''
Copyright (c) 2023 by Haiming Zhang. All Rights Reserved.

Author: Haiming Zhang
Date: 2023-08-30 10:43:44
Email: haimingzhang@link.cuhk.edu.cn
Description: 
'''


import torch
import os
import os.path as osp

from ckpt_utils import dump_state_dict


def load_fbocc_ckpt():
    ckpt_path = "/mnt/data3/zhanghm/Code/Occupancy/Occ_Challenge/ckpts/r50_256x705_depth_pretrain.pth"
    # ckpt_path = "/data/zhanghm/Code/FB-BEV/ckpts/resnet50-0676ba61.pth"
    ckpt = torch.load(ckpt_path, map_location='cpu')
    print(ckpt.keys())
    print(ckpt['epoch'], ckpt['updates'])

    state_dict = ckpt['state_dict'] if 'state_dict' in ckpt.keys() else ckpt
    # print(state_dict.keys())

    # dump_state_dict("r50_256x705_depth_pretrain.txt", state_dict, need_shape=True)
    # dump_state_dict("resnet50-0676ba61.txt", state_dict, need_shape=True)


def load_2dpass_ckpt():
    ckpt_path = "/home/zhanghm/2DPASS_6scale_256dim/epoch=69-step=61529.ckpt"
    ckpt = torch.load(ckpt_path, map_location='cpu')
    print(ckpt.keys())

    # torch.save(ckpt['state_dict'], "2DPASS_6scale_256dim_state_dict.pth")


def dump_ckpt():
    ckpt_path = "/mnt/data2/zhanghm/Code/Github/OccNet/ckpts/dd3d_det_final.pth"
    ckpt = torch.load(ckpt_path, map_location='cpu')

    state_dict = ckpt['state_dict'] if 'state_dict' in ckpt.keys() else ckpt

    # dump the results
    file_name = osp.basename(ckpt_path).replace(".pth", ".txt")
    save_dir = "./temp"
    os.makedirs(save_dir, exist_ok=True)

    file_path = osp.join(save_dir, file_name)
    dump_state_dict(file_path, state_dict, need_shape=True)


if __name__ == "__main__":
    dump_ckpt()