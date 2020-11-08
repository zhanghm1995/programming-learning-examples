import os

def read_kitti_oxts(self, dataset_root, sequence_num):
    oxts_file = os.path.join(dataset_root, 'oxts/{0:04d}'.format(int(sequence_num)))

    with open(oxts_file, 'r') as f:
        lines = f.readlines()
    
    oxts_dict = {}
    for i, line in enumerate(lines):
        oxt = line.strip().split(' ')
        oxts_dict[i] = oxt
    return oxts_dict  

with open("/mnt/study/Datasets/KITTI/tracking/training/oxts/0008.txt") as f:
    lines = f.readlines()

oxts_dict = {}
for i, line in enumerate(lines):
    oxt = line.strip().split(' ')
    oxts_dict[i] = oxt
    
print(oxts_dict)