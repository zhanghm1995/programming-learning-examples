'''
Copyright (c) 2023 by Haiming Zhang. All Rights Reserved.

Author: Haiming Zhang
Date: 2023-03-12 11:41:14
Email: haimingzhang@link.cuhk.edu.cn
Description: Learn the usage of DataLoader.
Reference: https://www.cnblogs.com/marsggbo/p/11308889.html
'''

import torch
from torch.utils.data import Dataset, DataLoader, SequentialSampler, RandomSampler


def test_sampler():
    # sampler = SequentialSampler(range(10))
    sampler = RandomSampler(range(10))
    print(sampler, sampler.__len__(), sampler.__iter__())
    for i in sampler:
        print(i)
    print(list(sampler))

    batch_sampler = torch.utils.data.BatchSampler(sampler, 3, drop_last=False)
    print(list(batch_sampler))
    for i in batch_sampler:
        print(i)



if __name__ == "__main__":
    test_sampler()

