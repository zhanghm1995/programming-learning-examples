'''
Copyright (c) 2023 by Haiming Zhang. All Rights Reserved.

Author: Haiming Zhang
Date: 2023-08-30 10:54:45
Email: haimingzhang@link.cuhk.edu.cn
Description: 
'''

import torch
import os
import os.path as osp


def dump_state_dict(file_path, state_dict, need_shape=False):
    content = []
    for k, v in state_dict.items():
        if need_shape:
            line = f"{k} {v.shape}"
        else:
            line = k
        content.append(line)
    
    with open(file_path, 'w') as fd:
        fd.writelines([i + '\n' for i in content])


def load_ckpt():
    ckpt_path = "/mnt/data2/zhanghm/Code/OCC_Distill/ckpts/dd3d_det_final.pth"
    state_dict = torch.load(ckpt_path)['state_dict']
    print(state_dict.keys())
    print(state_dict['pixel_mean'])
    print(state_dict['pixel_std'])


if __name__ == "__main__":
    load_ckpt()