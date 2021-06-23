from pathlib import Path
import os
from os import path as osp
from concurrent import futures as futures
import shutil
import numpy as np
import math

def mkdir_or_exist(dir_name, mode=0o777):
    if dir_name == '':
        return
    dir_name = osp.expanduser(dir_name)
    os.makedirs(dir_name, mode=mode, exist_ok=True)

def _read_imageset_file(path):
    with open(path, 'r') as f:
        lines = f.readlines()
    return [int(line) for line in lines]


def test_path(data_root):
    training_dir = Path(data_root)  / 'training'
    training_calib_dir = training_dir / 'calib'
    training_velodyne_dir = training_dir / 'velodyne'
    training_label_dir = training_dir / 'label_2'
 
    imageset_folder = Path(data_root) / 'ImageSets'
 
    val_img_ids = _read_imageset_file(str(imageset_folder / 'val.txt'))
    
    print(training_dir, type(training_dir))
    print(val_img_ids[:10])
 
    val_dir = Path(data_root) / 'validate'
    val_calib_dir = val_dir / 'calib'
    val_velodyne_dir = val_dir / 'velodyne'
    val_label_dir = val_dir / 'label_2'
    mkdir_or_exist(val_calib_dir)
    mkdir_or_exist(val_velodyne_dir)
    mkdir_or_exist(val_label_dir)

   
    def map_func(idx):
       src_calib_file = osp.join(training_calib_dir, f'{idx}.txt')
       src_velodyne_file = osp.join(training_velodyne_dir, f'{idx}.bin')
       src_label_file = os.path.join(training_label_dir, f'{idx}.txt')

       dst_calib_file = os.path.join(val_calib_dir, f'{idx}.txt')
       dst_velodyne_file = osp.join(val_velodyne_dir, f'{idx}.bin')
       dst_label_file = os.path.join(val_label_dir, f'{idx}.txt')

       shutil.copyfile(src_calib_file, dst_calib_file)
       shutil.copyfile(src_velodyne_file, dst_velodyne_file)
       shutil.copyfile(src_label_file, dst_label_file)
    with futures.ThreadPoolExecutor(8) as executor:
       executor.map(map_func, val_img_ids)