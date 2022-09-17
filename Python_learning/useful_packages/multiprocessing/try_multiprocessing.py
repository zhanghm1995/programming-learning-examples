'''
Copyright (c) 2022 by Haiming Zhang. All Rights Reserved.

Author: Haiming Zhang
Date: 2022-09-06 22:14:29
Email: haimingzhang@link.cuhk.edu.cn
Description: Have a try on multiprocessing package. 
We can use multiprocessing package to do the parallel computing in multiple processes, rather than multiple threads.
Reference: https://towardsdatascience.com/parallelism-with-python-part-1-196f0458ca14
'''

import time
import multiprocessing as mp
from multiprocessing import Pool
import itertools
from tqdm import tqdm


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


def dummy_func_multi_args(input, target_size, output_dir):
    print("Hello dummy_func_multi_args", input, target_size, output_dir)
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
    
    res = pool.imap_unordered(dummy_func, input)
    print(type(res))
    for i in res:
        print(i)


def starmap_multiprocess_multi_args(num_used_cpus):
    """starmap could receive multiple arguments.

    Args:
        num_used_cpus (_type_): _description_
    """
    pool = Pool(processes=num_used_cpus)
    input = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
    input = range(20)

    target_size = 512
    output_dir = "temp"

    function_parameters = zip(
        input,
        itertools.repeat(target_size),
        itertools.repeat(output_dir)
    )
    
    ## return the results in the order of input but return all results at one time finally
    res = pool.starmap(dummy_func_multi_args, function_parameters)
    print(type(res))
    for i in res:
        print(i)


def f(x):
    time.sleep(1)
    return x * x


def use_tqdm_with_multiprocess():
    """
    Reference: https://blog.csdn.net/weixin_39274659/article/details/107794635
    """
    with Pool(5) as p:
        print(list((tqdm(p.imap(f, range(10)), total=10, desc='Inspecting'))))


if __name__ == "__main__":
    imap_unordered_multiprocess(10)
    exit(0)

    use_tqdm_with_multiprocess()
    exit()

    num_cpus = mp.cpu_count()
    num_used_cpus = int(num_cpus * 0.25)

    print(f"Use {num_used_cpus} cpus..")

    imap_multiprocess(num_used_cpus)
    # imap_unordered_multiprocess(num_used_cpus)