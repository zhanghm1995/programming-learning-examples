import os
import numpy as np

def read_txt_to_numpy(file_path):
    a = np.loadtxt(file_path)
    print(a.shape, a.dtype)

def read_kitti_oxts(dataset_root, sequence_num):
    """
    read txt file line by line to a dictionary
    """
    oxts_file = os.path.join(dataset_root, 'oxts/{0:04d}.txt'.format(int(sequence_num)))

    with open(oxts_file, 'r') as f:
        lines = f.readlines()
    
    print(len(lines))
    oxts_dict = {}
    for i, line in enumerate(lines):
        oxt = line.strip().split(' ')
        oxts_dict[i] = oxt
    return oxts_dict  

if __name__ == '__main__':
    oxts_dict = read_kitti_oxts("data", 8)
    print(type(oxts_dict))

    read_txt_to_numpy("data/oxts/0008.txt")
    
