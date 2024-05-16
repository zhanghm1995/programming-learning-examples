'''
Copyright (c) 2024 by Haiming Zhang. All Rights Reserved.

Author: Haiming Zhang
Date: 2024-05-16 08:54:06
Email: haimingzhang@link.cuhk.edu.cn
Description: Use the multiprocessing function in mmengine package.
Reference: https://mmengine.readthedocs.io/zh-cn/latest/api/generated/mmengine.utils.track_parallel_progress.html
'''
import os
import os.path as osp
import numpy as np
import pickle
import torch
import mmengine
from functools import partial


def process_scene(path, fast, idx):
    print(f'Processing {idx}th scene in {path} with fast={fast}...')
    print(idx)
    res = f'{path}/scene_{idx}.pkl'
    return res


nproc = 4
path = 'data'
fast = True

results = mmengine.track_parallel_progress(
    func=partial(process_scene, path, fast),
    tasks=range(100),
    nproc=nproc,
    keep_order=True)
print(results[:10])
