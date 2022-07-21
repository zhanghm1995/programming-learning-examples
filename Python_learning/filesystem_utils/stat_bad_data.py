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
import yaml


def get_folder_list(root_dir):
    folder_list = [entry.path for entry in os.scandir(root_dir) if entry.is_dir()]
    folder_list = sorted(folder_list)
    return folder_list


def get_inconsitent_files(root_dir):
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


def count_unprocessed_files():
    data_root = "/mnt/inspurfs/user-fs/221019051/Datasets/Face/HDTF_preprocessed"

    folder_list = get_folder_list(data_root)

    print(len(folder_list), folder_list[:3])

    unprocessed_data_list = []
    whole_unprocessed_data_list = []

    ## 1) Read the yaml file to get total number of frames
    for folder in tqdm(folder_list):
        meta_info_fp = osp.join(folder, "meta_info.yaml")

        with open(meta_info_fp,'r') as f:
            meta_info = yaml.load(f, Loader=yaml.FullLoader)
        
        num_frames = meta_info["num_frames"]

        ## 2) Check whether parsing map exist
        candidate_check_fp = osp.join(folder, "face_image", "parsing_map")
        candidate_check_fp = osp.join(folder, "lower_mask")

        if not osp.exists(candidate_check_fp):
            whole_unprocessed_data_list.append(osp.basename(folder))
        else:
            ## 3) Check whether all the frames are processed
            # num_parsing_map = len(glob(osp.join(candidate_check_fp, "*.png")))
            num_candidate_files = len(os.listdir(candidate_check_fp))

            if num_frames != num_candidate_files:
                unprocessed_data_list.append(osp.basename(folder))

    print(len(unprocessed_data_list), unprocessed_data_list[:3])
    print(len(whole_unprocessed_data_list), whole_unprocessed_data_list[:3])
    
    with open("unprocessed_data_list.txt", 'w') as fd:
        fd.writelines([i + '\n' for i in unprocessed_data_list])


def stat_needed_folders():
    """Statistic the needed folders that you specified
    """
    data_root = "/home/zhanghm/Research/V100/TalkingFaceFormer/HDTF_preprocessed"
    folder_list = get_folder_list(data_root)

    print(len(folder_list), folder_list[:3])

    needed_folders_list = []
    
    for folder in tqdm(folder_list):
        mask_fg_path = osp.join(folder, "deep3dface_512")
        
        if not osp.exists(mask_fg_path):
            needed_folders_list.append(osp.basename(folder))
    
    print(len(needed_folders_list), needed_folders_list[:3])
    with open("needed_folders_list.txt", 'w') as fd:
        fd.writelines([i + '\n' for i in needed_folders_list])


if __name__ == "__main__":
    import fire
    fire.Fire()