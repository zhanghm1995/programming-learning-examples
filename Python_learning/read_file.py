import os

def read_kitti_oxts(dataset_root, sequence_num):
    """Learn how to read txt file
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
    dataset_root = "/media/zhanghm/Data/Datasets/KITTI/tracking/training/"
    oxts_dict = read_kitti_oxts(dataset_root, 1)