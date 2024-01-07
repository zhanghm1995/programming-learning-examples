'''
Copyright (c) 2023 by Haiming Zhang. All Rights Reserved.

Author: Haiming Zhang
Date: 2023-10-26 12:12:10
Email: haimingzhang@link.cuhk.edu.cn
Description: 
'''

import tqdm
import multiprocessing

def my_function(arg1, arg2, arg3):
  return arg1 + arg2 + arg3

def my_function_star(args):
    return my_function(*args)

jobs = 4
with multiprocessing.Pool(jobs) as pool:
    args = [(i, i, i) for i in range(10000)]
    results = list(tqdm.tqdm(pool.imap(my_function_star, args), total=len(args)))