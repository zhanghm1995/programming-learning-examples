'''
Copyright (c) 2023 by Haiming Zhang. All Rights Reserved.

Author: Haiming Zhang
Date: 2023-07-08 16:44:47
Email: haimingzhang@link.cuhk.edu.cn
Description: 
'''

import multiprocessing as mp

def get_num_of_cpus():
    num_cpus = mp.cpu_count()

    print("Number of processers: ", num_cpus)
    return num_cpus


if __name__ == "__main__":
    get_num_of_cpus()
