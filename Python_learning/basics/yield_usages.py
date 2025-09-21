'''
Copyright (c) 2022 by Haiming Zhang. All Rights Reserved.

Author: Haiming Zhang
Date: 2022-06-29 14:31:10
Email: haimingzhang@link.cuhk.edu.cn
Description: 
'''

import numpy as np
import itertools


data = itertools.islice('ABCDEFG', 2, None, 2)

for i in data:
    print(i)

exit()

def sample_data(loader, sampler=None):
    epoch = -1
    while True:
        epoch += 1
        if sampler is not None:
            sampler.set_epoch(epoch)
        for batch in loader:
            yield batch


class MyDataset(object):
    def __init__(self) -> None:
        super().__init__()
        self.data = list(range(10))

    def __iter__(self):
        for i, data in enumerate(self.data):
            yield i, data


dataset = MyDataset()
for i, data in dataset:
    print(i, data)


dl_iter = sample_data(dataset)
for j in range(20):
    i, data = next(dl_iter)
    print(j, i)