'''
Copyright (c) 2022 by Haiming Zhang. All Rights Reserved.

Author: Haiming Zhang
Date: 2022-09-06 22:14:29
Email: haimingzhang@link.cuhk.edu.cn
Description: Have a try on multiprocessing package. 
We can use multiprocessing package to do the parallel computing in multiple processes, rather than multiple threads.
'''

import multiprocessing as mp
from multiprocessing import Pool

def get_num_of_cpus():
    num_cpus = mp.cpu_count()

    print("Number of processers: ", num_cpus)
    return num_cpus


def get_start_methods():
    """
    Reference: https://stackoverflow.com/questions/43818519/what-is-the-meaning-of-context-argument-in-multiprocessing-pool-pool
    """
    all_start_methods = mp.get_all_start_methods()
    print(all_start_methods)


def dummy_func(input):
    print("Hello world", input)
    return input + 100


def imap_multiprocess(num_used_cpus):
    pool = Pool(processes=num_used_cpus)
    input = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
    input = range(20)
    
    iter = pool.imap(dummy_func, input)
    for i in iter:
        print(i)


def imap_unordered_multiprocess(num_used_cpus):
    pool = Pool(processes=num_used_cpus)
    input = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
    input = range(20)
    
    iter = pool.imap_unordered(dummy_func, input)
    for i in iter:
        print(i)



if __name__ == "__main__":
    get_start_methods()
    exit()

    num_cpus = mp.cpu_count()
    num_used_cpus = int(num_cpus * 0.25)

    print(f"Use {num_used_cpus} cpus..")

    imap_multiprocess(num_used_cpus)
    # imap_unordered_multiprocess(num_used_cpus)